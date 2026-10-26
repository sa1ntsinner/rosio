from dataclasses import dataclass, field


@dataclass
class Finding:
    check: str
    topic: str
    severity: str
    message: str
    evidence: dict = field(default_factory=dict)


@dataclass
class TopicSeries:
    topic: str
    msgtype: str
    stamps: list[float]
    receive_times: list[float]
