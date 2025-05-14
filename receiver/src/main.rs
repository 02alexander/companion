use std::io::Read;
use common::Message;

fn connect() -> anyhow::Result<()> {

    let address = "192.168.0.132:1234";
    println!("Connecting to {address:?}");
    let mut connection = std::net::TcpStream::connect(address)?;
    println!("Connected!");
    
    let mut size_bytes = [0 as u8; 2];
    let mut msg_buffer = [0 as u8; 128];
    connection.set_read_timeout(Some(std::time::Duration::from_millis(1000)))?;
    loop {
        connection.read(&mut size_bytes)?;
        let n = u16::from_be_bytes(size_bytes);
        connection.read(&mut msg_buffer[..n as usize])?;
        
        let (msg, _) = bincode::serde::decode_from_slice::<Message, _>(&msg_buffer[..n as usize], bincode::config::standard())?;
        
        println!("{:?}", msg.ticks);
    }

    Ok(())
}

fn main() {
    loop {
        if let Err(e) = connect() {
            println!("Error: {e:?}", );
        }
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
