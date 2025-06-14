use common::{LogMessage};
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
    connection.set_read_timeout(Some(std::time::Duration::from_millis(3000)))?;

    loop {
        connection.read(&mut size_bytes)?;
        let n = u16::from_be_bytes(size_bytes);
        if n as usize > msg_buffer.len() {
            continue;
        }

        connection.read(&mut msg_buffer[..n as usize])?;
        
        let (msg, _) = bincode::serde::decode_from_slice::<LogMessage, _>(
            &msg_buffer[..n as usize],
            bincode::config::standard(),
        )?;


        match msg {
            LogMessage::Controller(msg) => {
                // rec.set_time_sequence("sample", sample);
                let time_ms = msg.time_ms;
        
                rec.set_time("sample_time", Duration::from_millis(time_ms));
                
                rec.log("control", &rerun::Scalars::single(msg.control as f64)).unwrap();
                rec.log("sensor_velocity", &rerun::Scalars::single(msg.sensor_wheel_velocity as f64)).unwrap();
                rec.log("wheel_velocity", &rerun::Scalars::single(msg.wheel_velocity as f64)).unwrap();
                rec.log("sensor_pend_angle", &rerun::Scalars::single(msg.sensor_pend_angle as f64)).unwrap();
                rec.log("pend_angle", &rerun::Scalars::single(msg.pend_angle as f64)).unwrap();
                rec.log("pend_velocity", &rerun::Scalars::single(msg.pend_velocity as f64)).unwrap();
            },
            LogMessage::Bench(msg) => {
                let time_ms = msg.time_ms;
                rec.set_time("sample_time", Duration::from_millis(time_ms));
                rec.log("control", &rerun::Scalars::single(msg.control as f64)).unwrap();
                rec.log("wheel_velocity", &rerun::Scalars::single(msg.signed_rot_speed as f64)).unwrap();
                rec.log("abs_wheel_velocity", &rerun::Scalars::single(msg.abs_rot_speed as f64)).unwrap();
            }
            LogMessage::Alive => {}
        }
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

