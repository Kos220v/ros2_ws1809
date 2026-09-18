# -*- coding: utf-8 -*-
"""Заглушка rclpy.node.Node: параметры, паблишеры, подписки, таймеры."""


class _Param:
    def __init__(self, value):
        self.value = value


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Logger:
    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


class _Stamp:
    """Заглушка builtin_interfaces/msg/Time."""

    def __init__(self, nanoseconds=0):
        self.sec = int(nanoseconds // 1_000_000_000)
        self.nanosec = int(nanoseconds % 1_000_000_000)


class _Clock:
    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        return self

    def to_msg(self):
        return _Stamp(self.nanoseconds)


class Node:
    def __init__(self, name, param_overrides=None, **kwargs):
        self._name = name
        self._param_overrides = dict(param_overrides or {})
        self._params = {}
        self._pubs = {}
        self.subs = {}
        self.timers = []
        self._clock = _Clock()
        self._logger = _Logger()

    # -- параметры ---------------------------------------------------------
    def declare_parameter(self, name, default):
        value = self._param_overrides.get(name, default)
        self._params[name] = _Param(value)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    # -- топики ------------------------------------------------------------
    def create_publisher(self, msg_type, topic, qos):
        pub = _Publisher(topic)
        self._pubs[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs.setdefault(topic, []).append(cb)
        return None

    def create_timer(self, period, cb):
        self.timers.append(cb)
        return None

    # -- прочее ------------------------------------------------------------
    def get_clock(self):
        return self._clock

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        return None

    # -- хелперы тестов (в реальном rclpy их нет) ---------------------------
    def feed(self, topic, msg):
        for cb in self.subs[topic]:
            cb(msg)

    def pubs(self):
        return self._pubs
