import rosio_monitor.monitor_node as monitor_node


def test_main_is_callable():
    assert callable(monitor_node.main)


def test_monitor_node_class_exists():
    assert hasattr(monitor_node, "MonitorNode")
