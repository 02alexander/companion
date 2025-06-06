use common::LogMessage;
use std::{io::Read, time::Duration};

fn connect() -> anyhow::Result<()> {
    let address = "192.168.0.132:1234";
    println!("Connecting to {address:?}");
    let mut connection = std::net::TcpStream::connect(address)?;
    println!("Connected!");

    let rec = rerun::RecordingStreamBuilder::new("companion")
        .connect_grpc()
        .unwrap();

    let mut size_bytes = [0 as u8; 2];
    let mut msg_buffer = [0 as u8; 128];
    connection.set_read_timeout(Some(std::time::Duration::from_millis(1000)))?;

    loop {
        connection.read(&mut size_bytes)?;
        let n = u16::from_be_bytes(size_bytes);
        connection.read(&mut msg_buffer[..n as usize])?;

        let (msg, _) = bincode::serde::decode_from_slice::<LogMessage, _>(
            &msg_buffer[..n as usize],
            bincode::config::standard(),
        )?;

        // rec.set_time_sequence("sample", sample);
        let time_ms = msg.time_ms;

        rec.set_time("sample_time", Duration::from_millis(time_ms));
        
        rec.log("wheel_velocity", &rerun::Scalars::single(msg.wheel_velocity as f64)).unwrap();
        rec.log("angle", &rerun::Scalars::single(msg.pend_angle as f64)).unwrap();
        rec.log("control", &rerun::Scalars::single(msg.control as f64)).unwrap();
    }

    // static gain: 2050 / 0.125
    // Time constant = 0.56
}

fn main() {
    loop {
        if let Err(e) = connect() {
            println!("Error: {e:?}",);
        }
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
