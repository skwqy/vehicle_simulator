//! TCP client to YFAOS `AgvSocketServer` — one connection per AGV, same framing as FV2 `SocketPlcBridge`.

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use log::{info, warn};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::TcpStream;
use tokio::sync::Mutex;

use crate::config::Config;
use crate::map::MapModel;
use crate::vehicle_simulator::VehicleSimulator;
use crate::socket::framing::{
    normalize_downlink_suffix, parse_frame_body, to_short_type_name, wrap, ParsedFrame,
};

async fn read_tcp_body(stream: &mut OwnedReadHalf, max_len: usize) -> std::io::Result<Vec<u8>> {
    let mut len_b = [0u8; 4];
    stream.read_exact(&mut len_b).await?;
    let len = u32::from_be_bytes(len_b) as usize;
    if len == 0 || len > max_len {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("invalid frame length: {len}"),
        ));
    }
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf).await?;
    Ok(buf)
}

async fn write_tcp_frame(stream: &mut OwnedWriteHalf, body: &[u8]) -> std::io::Result<()> {
    stream.write_u32(body.len() as u32).await?;
    stream.write_all(body).await?;
    stream.flush().await?;
    Ok(())
}

fn physics_tick_ms(cfg: &Config) -> u64 {
    let status = cfg.simulation.status_interval_ms;
    let moving = cfg.simulation.status_interval_moving_ms;
    let step = if moving > 0 {
        status.min(moving)
    } else {
        status.min(50)
    };
    step.max(20)
}

/// One attempt to connect to the scheduler; logs and returns `None` on failure or timeout.
async fn tcp_connect_to_scheduler(
    addr: SocketAddr,
    connect_timeout_ms: u64,
    vehicle_tag: &str,
) -> Option<TcpStream> {
    if connect_timeout_ms > 0 {
        match tokio::time::timeout(
            Duration::from_millis(connect_timeout_ms),
            TcpStream::connect(addr),
        )
        .await
        {
            Ok(Ok(stream)) => Some(stream),
            Ok(Err(e)) => {
                warn!(
                    target: "socket",
                    "[{}] NOT CONNECTED — connect failed to {}: {e}",
                    vehicle_tag,
                    addr
                );
                None
            }
            Err(_) => {
                warn!(
                    target: "socket",
                    "[{}] NOT CONNECTED — connect timeout to {} after {} ms",
                    vehicle_tag,
                    addr,
                    connect_timeout_ms
                );
                None
            }
        }
    } else {
        match TcpStream::connect(addr).await {
            Ok(stream) => Some(stream),
            Err(e) => {
                warn!(
                    target: "socket",
                    "[{}] NOT CONNECTED — connect failed to {}: {e}",
                    vehicle_tag,
                    addr
                );
                None
            }
        }
    }
}

/// Background task: advance physics on an interval and publish `StatusMsg` when due.
fn spawn_physics_status_tick_task(
    write: Arc<Mutex<OwnedWriteHalf>>,
    engine: Arc<Mutex<VehicleSimulator>>,
    tick_ms: u64,
    sim_dt: f32,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        let mut ticker = tokio::time::interval(Duration::from_millis(tick_ms));
        loop {
            ticker.tick().await;
            let mut eng = engine.lock().await;
            eng.tick(sim_dt);
            if !eng.should_publish_status() {
                continue;
            }
            let pb = eng.build_status_payload();
            drop(eng);
            let body = wrap("StatusMsg", &pb);
            let mut w = write.lock().await;
            if write_tcp_frame(&mut *w, &body).await.is_err() {
                break;
            }
            drop(w);
            engine.lock().await.after_status_published();
        }
    })
}

/// Read MC → AGV frames, drive the simulator, and write replies (and follow-up status) on the same TCP connection.
async fn session_downlink_loop(
    mut read_half: OwnedReadHalf,
    max_frame: usize,
    uplink_crc: bool,
    engine: Arc<Mutex<VehicleSimulator>>,
    write: Arc<Mutex<OwnedWriteHalf>>,
    vehicle_tag: &str,
    vehicle_log_target: &str,
) -> std::io::Result<()> {
    loop {
        let body = match read_tcp_body(&mut read_half, max_frame).await {
            Ok(b) => b,
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e),
        };
        let parsed = match parse_frame_body(&body, uplink_crc) {
            Ok(p) => p,
            Err(e) => {
                warn!(target: "socket", "[{}] parse frame failed: {e}", vehicle_tag);
                continue;
            }
        };
        let ParsedFrame {
            full_type_name,
            protobuf,
            crc_ok,
        } = parsed;
        if !crc_ok {
            warn!(
                target: "socket",
                "[{}] CRC mismatch fullType={full_type_name} (continuing)",
                vehicle_tag
            );
        }
        let short = to_short_type_name(&full_type_name);
        let normalized = normalize_downlink_suffix(&short);
        info!(
            target: vehicle_log_target,
            "[{}] MC -> AGV ({}): received",
            vehicle_tag,
            short
        );
        let replies = {
            let mut eng = engine.lock().await;
            eng.handle_downlink(&normalized, &protobuf)
        };
        {
            let mut w = write.lock().await;
            for (msg_type, payload) in replies {
                let b = wrap(&msg_type, &payload);
                write_tcp_frame(&mut *w, &b).await?;
                info!(
                    target: vehicle_log_target,
                    "[{}] AGV -> MC ({}): sent",
                    vehicle_tag,
                    msg_type
                );
            }
        }
        let mut eng = engine.lock().await;
        if eng.should_publish_status() {
            let pb = eng.build_status_payload();
            drop(eng);
            let b = wrap("StatusMsg", &pb);
            let mut w = write.lock().await;
            write_tcp_frame(&mut *w, &b).await?;
            drop(w);
            engine.lock().await.after_status_published();
            info!(
                target: vehicle_log_target,
                "[{}] AGV -> MC (StatusMsg): sent (after downlink)",
                vehicle_tag
            );
        }
    }
}

async fn run_session(
    stream: TcpStream,
    engine: Arc<Mutex<VehicleSimulator>>,
    cfg: Arc<Config>,
    vehicle_tag: &str,
    vehicle_log_target: &str,
) -> std::io::Result<()> {
    let uplink_crc = cfg.socket.uplink_has_crc;
    let max_frame = cfg.socket.max_frame_length;
    let sim_dt = cfg.map.sim_dt_seconds;

    let (read_half, write_half) = stream.into_split();
    let write = Arc::new(Mutex::new(write_half));

    {
        let mut eng = engine.lock().await;
        let pb = eng.build_status_payload();
        drop(eng);
        let body = wrap("StatusMsg", &pb);
        let mut w = write.lock().await;
        write_tcp_frame(&mut *w, &body).await?;
        engine.lock().await.after_status_published();
    }
    info!(
        target: vehicle_log_target,
        "[{}] AGV -> MC (StatusMsg): initial after connect",
        vehicle_tag
    );

    let tick_task = spawn_physics_status_tick_task(
        Arc::clone(&write),
        Arc::clone(&engine),
        physics_tick_ms(&cfg),
        sim_dt,
    );

    let result = session_downlink_loop(
        read_half,
        max_frame,
        uplink_crc,
        engine,
        write,
        vehicle_tag,
        vehicle_log_target,
    )
    .await;

    tick_task.abort();
    result
}

pub async fn connection_loop(
    cfg: Arc<Config>,
    agv_id: i32,
    robot_index: u32,
    map: Option<Arc<MapModel>>,
    log_target: String,
) {
    let vehicle_tag = format!("agv_id={} serial={}", agv_id, cfg.vehicle.serial_number);

    let addr: SocketAddr = match format!("{}:{}", cfg.socket.host, cfg.socket.port).parse() {
        Ok(a) => a,
        Err(e) => {
            tracing::error!(
                target: "bootstrap",
                "[{}] invalid socket address {}:{}: {e}",
                vehicle_tag,
                cfg.socket.host,
                cfg.socket.port
            );
            return;
        }
    };

    let mut running = true;
    while running {
        let stream = tcp_connect_to_scheduler(
            addr,
            cfg.socket.connect_timeout_ms,
            vehicle_tag.as_str(),
        )
        .await;

        let Some(stream) = stream else {
            if !cfg.socket.reconnect_enabled {
                break;
            }
            tokio::time::sleep(Duration::from_millis(cfg.socket.reconnect_interval_ms.max(500)))
                .await;
            continue;
        };

        let _ = stream.set_nodelay(true);

        let engine = Arc::new(Mutex::new(VehicleSimulator::new(
            (*cfg).clone(),
            agv_id,
            robot_index,
            map.clone(),
            log_target.clone(),
        )));
        let cfg_c = Arc::clone(&cfg);
        let serial = cfg.vehicle.serial_number.as_str();
        info!(
            target: log_target.as_str(),
            "CONNECTED — serial_number={} agv_id={} server={} (TCP to scheduler, protobuf framing)",
            serial,
            agv_id,
            addr
        );

        match run_session(
            stream,
            Arc::clone(&engine),
            cfg_c,
            vehicle_tag.as_str(),
            log_target.as_str(),
        )
        .await
        {
            Ok(()) => info!(
                target: log_target.as_str(),
                "[{}] DISCONNECTED — scheduler closed TCP session (was connected to {})",
                vehicle_tag,
                addr
            ),
            Err(e) => warn!(
                target: log_target.as_str(),
                "[{}] DISCONNECTED — session error from {}: {e}",
                vehicle_tag,
                addr
            ),
        }

        if !cfg.socket.reconnect_enabled {
            running = false;
        } else {
            tokio::time::sleep(Duration::from_millis(cfg.socket.reconnect_interval_ms.max(500)))
                .await;
        }
    }
}
