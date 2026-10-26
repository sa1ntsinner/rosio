fn main() {
    let (connection, threads) = lsp_server::Connection::stdio();
    if let Err(error) = rosio_lsp::server::run(connection) {
        eprintln!("rosio-lsp: {error}");
    }
    if let Err(error) = threads.join() {
        eprintln!("rosio-lsp: {error}");
    }
}
