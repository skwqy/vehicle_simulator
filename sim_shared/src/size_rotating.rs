//! Fixed-size log rotation: current file + numbered backups, max total file count, overwrite oldest.

use std::fs::{self, File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct SizeRotatingFile {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    base_path: PathBuf,
    max_bytes: u64,
    max_files: usize,
    file: File,
    current_size: u64,
}

impl SizeRotatingFile {
    pub fn new(base_path: PathBuf, max_bytes: u64, max_files: usize) -> io::Result<Self> {
        if max_files < 1 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "max_files must be at least 1",
            ));
        }
        if max_bytes == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "max_bytes must be positive",
            ));
        }
        let (file, size) = open_append(&base_path)?;
        Ok(Self {
            inner: Arc::new(Mutex::new(Inner {
                base_path,
                max_bytes,
                max_files,
                file,
                current_size: size,
            })),
        })
    }
}

fn open_append(path: &Path) -> io::Result<(File, u64)> {
    let file = OpenOptions::new().create(true).append(true).open(path)?;
    let len = file.metadata()?.len();
    Ok((file, len))
}

impl Inner {
    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?;

        let base = &self.base_path;
        if self.max_files <= 1 {
            self.file = OpenOptions::new()
                .create(true)
                .write(true)
                .truncate(true)
                .open(base)?;
            self.current_size = 0;
            return Ok(());
        }

        let last_idx = self.max_files.saturating_sub(1);
        if last_idx > 0 {
            let last = format!("{}.{}", base.display(), last_idx);
            let _ = fs::remove_file(&last);

            for i in (1..last_idx).rev() {
                let from = format!("{}.{}", base.display(), i);
                let to = format!("{}.{}", base.display(), i + 1);
                if Path::new(&from).exists() {
                    fs::rename(&from, &to)?;
                }
            }
        }

        let first_backup = format!("{}.1", base.display());
        if base.exists() {
            fs::rename(base, &first_backup)?;
        }

        let (file, size) = open_append(base)?;
        self.file = file;
        self.current_size = size;
        Ok(())
    }

    fn write_inner(&mut self, buf: &[u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }

        if self.current_size > 0 && self.current_size + buf.len() as u64 > self.max_bytes {
            self.rotate()?;
        }

        let n = self.file.write(buf)?;
        self.current_size += n as u64;
        Ok(n)
    }
}

impl Write for SizeRotatingFile {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.inner
            .lock()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?
            .write_inner(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        self.inner
            .lock()
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?
            .file
            .flush()
    }
}
