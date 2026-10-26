auto scan = create_subscription<Scan>("scan", 10, callback);
auto output = create_publisher<Scan>("output", 10);
auto a = create_subscription<Scan>("scan", 10, callback); auto b = create_publisher<Scan>("output", 10);
