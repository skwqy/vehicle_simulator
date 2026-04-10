use futures_util::{pin_mut, StreamExt};
use paho_mqtt as mqtt;
use std::sync::Arc;
use std::time::Instant;
use std::{process, time::Duration};
use tokio::sync::Mutex;
use log::{info, warn};

use crate::config;
use crate::mqtt_utils;
use crate::utils;
use crate::vehicle_simulator::VehicleSimulator;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_order::Order;
use crate::protocol::vda_2_0_0::vda5050_2_0_0_instant_actions::InstantActions;

pub async fn subscribe_vda_messages(
    config: config::Config,
    simulator: Arc<Mutex<VehicleSimulator>>,
    log_target: String,
) {
    let base_topic = format!(
        "{}/{}/{}/{}",
        config.mqtt_broker.vda_interface,
        config.vehicle.vda_version,
        config.vehicle.manufacturer,
        config.vehicle.serial_number,
    );

    let topics = vec![
        format!("{}/order", base_topic),
        format!("{}/instantActions", base_topic),
    ];

    if topics.is_empty() {
        tracing::error!(target: "bootstrap", "Error: No topics specified!");
        process::exit(-1);
    }

    // VDA 6.2: order / instantActions use QoS 0
    let qos = vec![0; topics.len()];
    let mut mqtt_client = create_mqtt_client();
    let message_stream = mqtt_client.get_stream(25);

    connect_to_broker(&mqtt_client).await;
    info!(
        target: log_target.as_str(),
        "MQTT subscriber connected to tcp://{}:{}",
        config.mqtt_broker.host,
        config.mqtt_broker.port
    );

    subscribe_to_topics(&mqtt_client, &topics, &qos).await;
    info!(
        target: log_target.as_str(),
        "MQTT subscribe OK — MC→AGV topics: {:?} (QoS {:?})",
        topics,
        qos
    );
    info!(
        target: log_target.as_str(),
        "Waiting for messages on subscribed topics (order, instantActions)"
    );

    pin_mut!(message_stream);
    while let Some(msg_opt) = message_stream.next().await {
        if let Some(msg) = msg_opt {
            handle_incoming_message(msg, &simulator, log_target.as_str()).await;
        } else {
            handle_connection_loss(&mqtt_client).await;
        }
    }
}

pub async fn publish_vda_messages(
    simulator: Arc<Mutex<VehicleSimulator>>,
    state_max_interval_secs: u64,
    sim_dt_seconds: f32,
    visualization_frequency: u64,
    log_target: String,
) {
    let mqtt_client = create_mqtt_client();
    connect_to_broker(&mqtt_client).await;
    let broker = config::get_config().mqtt_broker;
    info!(
        target: log_target.as_str(),
        "MQTT publisher connected to tcp://{}:{}",
        broker.host,
        broker.port
    );
    {
        let sim = simulator.lock().await;
        let (conn_tp, state_tp, viz_tp) = sim.vda_publish_topic_paths();
        info!(
            target: log_target.as_str(),
            "AGV publish topics registered (VDA) — connection: {}, state: {}, visualization: {}",
            conn_tp,
            state_tp,
            viz_tp
        );
    }

    // Publish initial connection (QoS 1)
    simulator.lock().await.publish_connection(&mqtt_client).await;

    // First state after ONLINE (VDA: MC should see state promptly)
    {
        let mut sim = simulator.lock().await;
        sim.publish_state(&mqtt_client).await;
        let _ = sim.take_state_publish_pending();
    }

    let sim_period_ms = ((sim_dt_seconds * 1000.0).round() as u64).max(10);
    let heartbeat = Duration::from_secs(state_max_interval_secs.max(1));
    let mut last_state_at = Instant::now();

    let viz_interval_ms = if visualization_frequency == 0 {
        u64::MAX
    } else {
        (1000u64 / visualization_frequency).max(1)
    };
    let mut viz_accum_ms: u64 = 0;

    loop {
        {
            let mut sim = simulator.lock().await;
            sim.update_state();

            let need_event = sim.take_state_publish_pending();
            let need_heartbeat = last_state_at.elapsed() >= heartbeat;
            if need_event || need_heartbeat {
                sim.publish_state(&mqtt_client).await;
                last_state_at = Instant::now();
            }
        }

        viz_accum_ms = viz_accum_ms.saturating_add(sim_period_ms);
        if viz_accum_ms >= viz_interval_ms {
            viz_accum_ms = 0;
            simulator.lock().await.publish_visualization(&mqtt_client).await;
        }

        tokio::time::sleep(Duration::from_millis(sim_period_ms)).await;
    }
}

fn create_mqtt_client() -> mqtt::AsyncClient {
    mqtt::AsyncClient::new(mqtt_utils::mqtt_create_opts()).unwrap_or_else(|e| {
        tracing::error!(target: "mqtt", "Error creating MQTT client: {:?}", e);
        process::exit(-1);
    })
}

async fn connect_to_broker(mqtt_client: &mqtt::AsyncClient) {
    let broker_config = config::get_config().mqtt_broker;
    let conn_opts = {
        let mut builder = mqtt::ConnectOptionsBuilder::with_mqtt_version(mqtt::MQTT_VERSION_5);
        builder.clean_start(true);
        if let (Some(username), Some(password)) = (&broker_config.username, &broker_config.password) {
            builder.user_name(username);
            builder.password(password.as_str());
        }
        builder.finalize()
    };
    mqtt_client.connect(conn_opts).await.unwrap();
}

async fn subscribe_to_topics(
    mqtt_client: &mqtt::AsyncClient,
    topics: &[String],
    qos: &[i32],
) {
    mqtt_client.subscribe_many(topics, qos).await.unwrap();
}

async fn handle_incoming_message(
    msg: mqtt::Message,
    simulator: &Arc<Mutex<VehicleSimulator>>,
    log_target: &str,
) {
    if msg.retained() {
        print!("(R) ");
    }

    let topic = msg.topic();
    let topic_type = utils::get_topic_type(topic);
    let payload = String::from_utf8_lossy(msg.payload()).to_string();

    info!(
        target: log_target,
        "MC -> {} ({}): {}",
        topic,
        topic_type,
        payload
    );

    match topic_type.as_ref() {
        "order" => handle_order_message(&payload, simulator, log_target).await,
        "instantActions" => handle_instant_actions_message(&payload, simulator, log_target).await,
        _ => warn!(target: log_target, "Unknown topic type: {}", topic_type),
    }
}

async fn handle_order_message(
    payload: &str,
    simulator: &Arc<Mutex<VehicleSimulator>>,
    log_target: &str,
) {
    match serde_json::from_str::<Order>(payload) {
        Ok(order) => {
            simulator.lock().await.process_order(order);
        }
        Err(e) => {
            warn!(
                target: log_target,
                "Reject order JSON (parse error): {e}"
            );
            tracing::warn!(target: "bootstrap", "Error parsing order message: {}", e);
        }
    }
}

async fn handle_instant_actions_message(
    payload: &str,
    simulator: &Arc<Mutex<VehicleSimulator>>,
    log_target: &str,
) {
    match serde_json::from_str::<InstantActions>(payload) {
        Ok(instant_actions) => {
            simulator.lock().await.accept_instant_actions(instant_actions);
        }
        Err(e) => {
            warn!(
                target: log_target,
                "Reject instantActions JSON (parse error): {e}"
            );
            tracing::warn!(target: "bootstrap", "Error parsing instant actions message: {}", e);
        }
    }
}

async fn handle_connection_loss(mqtt_client: &mqtt::AsyncClient) {
    tracing::warn!(target: "mqtt", "Lost connection. Attempting to reconnect...");
    
    while let Err(err) = mqtt_client.reconnect().await {
        tracing::warn!(target: "mqtt", "Error reconnecting: {}", err);
        tokio::time::sleep(Duration::from_millis(1000)).await;
    }
    
    tracing::info!(target: "mqtt", "Successfully reconnected to MQTT broker");
} 