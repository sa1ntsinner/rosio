"""Time source polling and grading.

Every poller returns an Observation, never a bare None for a failure:

    {'state': str, 'data': dict, 'detail': str}

state is one of:
    'ok'                 the tool ran and its output was understood, data is filled
    'absent'             the binary or the service is not present, a normal condition
    'permission_denied'  the tool ran but was not allowed to read the clock state
    'failed'             the tool ran and errored, detail carries the first stderr line
    'unparseable'        the tool exited 0 but nothing usable came out, which
                         includes a required number that is not a number

poll() returns {'ptp': Observation|None, 'chrony': Observation|None,
'timesyncd': Observation|None} where None means not attempted, for instance
timesyncd skipped because chrony answered.

assess() grades every source against the whole observation set, not each source
in isolation: a ptp port in MASTER or GRAND_MASTER only serves a trustworthy
epoch if the system clock under it is disciplined, so it is OK with a healthy
chrony or timesyncd observation and a WARN without one. An undisciplined master
is worse than no master, every sensor locks tightly to the wrong time.

assess() grades 'ok' on the data, and grades permission_denied, failed and
unparseable at WARN or worse with the detail in the message. An absent source
is fine on its own, but when no source was observed at all the answer is a
single WARN summary: an unmonitored clock is unknown, not healthy. Set
expect_none for hosts that genuinely run no sync daemon, either as the
'expect_none' key of the thresholds dict (canonical, so a node can pass it as
one parameter bundle) or as the assess(expect_none=True) argument, which wins
when given.

The runner returns (returncode, stdout, stderr) and raises FileNotFoundError
when the program does not exist.
"""

import subprocess

OK = 0
WARN = 1
ERROR = 2

PMC_PATHS = ("/usr/sbin/pmc", "pmc")
DEFAULT_PMC_COMMAND = ("pmc",)
PMC_COMMANDS = ("GET TIME_STATUS_NP", "GET PORT_DATA_SET")
COMMAND_TIMEOUT_S = 2.0

SOURCE_ORDER = ("ptp", "chrony", "timesyncd")

HEALTHY_PORT_STATES = ("SLAVE", "MASTER", "GRAND_MASTER")
SERVING_PORT_STATES = ("MASTER", "GRAND_MASTER")
SYSTEM_CLOCK_SOURCES = ("chrony", "timesyncd")
CHRONY_NORMAL_LEAP = "normal"
CHRONY_UNSYNCHRONIZED_LEAP = "not synchronised"

DEFAULT_THRESHOLDS = {
    "ptp_offset_warn_ns": 1000000,
    "ptp_offset_error_ns": 10000000,
    "chrony_offset_warn_s": 0.1,
}

DEFAULT_CONFIG = {
    "ptp_domain": 0,
    "pmc_socket": "",
    "pmc_command": DEFAULT_PMC_COMMAND,
}

PERMISSION_MARKERS = (
    "permission denied",
    "operation not permitted",
    "access denied",
    "must be root",
)

ABSENT_MARKERS = (
    "cannot talk to daemon",
    "timesync1.service not found",
    "unit not found",
    "could not be found",
)

STATE_OK = "ok"
STATE_ABSENT = "absent"
STATE_PERMISSION_DENIED = "permission_denied"
STATE_FAILED = "failed"
STATE_UNPARSEABLE = "unparseable"

PROBLEM_STATES = (STATE_PERMISSION_DENIED, STATE_FAILED, STATE_UNPARSEABLE)


def observation(state, data=None, detail=""):
    return {"state": state, "data": data or {}, "detail": detail}


def run_command(argv):
    try:
        done = subprocess.run(list(argv), capture_output=True, text=True, timeout=COMMAND_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return -1, "", f"timed out after {COMMAND_TIMEOUT_S:.0f} s"
    except subprocess.SubprocessError as error:
        return -1, "", str(error)
    return done.returncode, done.stdout, done.stderr


def _call(runner, argv):
    """Run argv, or return None when the program is not installed."""
    try:
        return runner(argv)
    except FileNotFoundError:
        return None
    except (OSError, subprocess.SubprocessError) as error:
        return -1, "", str(error)


def _first_line(text):
    for line in (text or "").splitlines():
        line = line.strip()
        if line:
            return line
    return ""


def _mentions(text, markers):
    lowered = (text or "").lower()
    return any(marker in lowered for marker in markers)


def _failure(code, stdout, stderr, absent_markers=()):
    """Turn a nonzero exit into permission_denied, absent or failed."""
    detail = _first_line(stderr) or _first_line(stdout)
    both = "{}\n{}".format(stderr or "", stdout or "")
    if _mentions(both, PERMISSION_MARKERS):
        return observation(STATE_PERMISSION_DENIED, detail=detail)
    if absent_markers and _mentions(both, absent_markers):
        return observation(STATE_ABSENT, detail=detail)
    return observation(STATE_FAILED, detail=detail or f"exit status {code}")


def _key_value_lines(text):
    """pmc and timedatectl both answer with `key value` or `key=value` lines."""
    pairs = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        key, separator, value = line.partition("=")
        if not separator:
            key, separator, value = line.partition(" ")
        if separator:
            pairs.append((key.strip(), value.strip()))
    return pairs


def _to_int(text):
    try:
        return int(text)
    except ValueError:
        return None


def _to_float(text):
    try:
        return float(text)
    except ValueError:
        return None


class _UnreadableFieldError(Exception):
    """A required numeric field was there but is not a number."""


def _required(name, text, convert):
    value = convert(text)
    if value is None:
        raise _UnreadableFieldError(f"{name} is not a number: {text!r}")
    return value


def _preferred_port_state(states):
    for wanted in HEALTHY_PORT_STATES:
        if wanted in states:
            return wanted
    return states[0]


def pmc_argv(command, config=None):
    """command is the argv prefix, the query arguments are appended to it."""
    settings = dict(DEFAULT_CONFIG)
    settings.update(config or {})
    prefix = [command] if isinstance(command, str) else list(command)
    argv = [*prefix, "-u", "-b", "0", "-d", str(int(settings.get("ptp_domain") or 0))]
    socket = settings.get("pmc_socket")
    if socket:
        argv += ["-s", str(socket)]
    return (*argv, *PMC_COMMANDS)


def pmc_prefixes(config=None):
    """The configured command verbatim, or the sbin path then the bare name."""
    command = (config or {}).get("pmc_command") or DEFAULT_PMC_COMMAND
    command = [str(part) for part in command]
    if tuple(command) == DEFAULT_PMC_COMMAND:
        return [[path] for path in PMC_PATHS]
    return [command]


def ptp_status(runner=run_command, config=None):
    """pmc is tried at its sbin path first, the bare name is the fallback."""
    for prefix in pmc_prefixes(config):
        answer = _call(runner, pmc_argv(prefix, config))
        if answer is None:
            continue
        code, stdout, stderr = answer
        if code != 0:
            if _is_pmc_socket_refusal(code, stdout, stderr):
                return observation(
                    STATE_PERMISSION_DENIED, detail=_first_line(stderr) or _first_line(stdout)
                )
            return _failure(code, stdout, stderr)
        try:
            data = _parse_pmc(stdout)
        except _UnreadableFieldError as error:
            return observation(STATE_UNPARSEABLE, detail=str(error))
        if data is None:
            return observation(
                STATE_UNPARSEABLE,
                detail=_first_line(stdout) or _first_line(stderr) or "pmc printed nothing usable",
            )
        return observation(STATE_OK, data)
    return observation(STATE_ABSENT, detail="pmc is not installed")


def _is_pmc_socket_refusal(code, stdout, stderr):
    """pmc exits 255 and complains about its uds socket when it may not bind."""
    return code == 255 and _mentions("{}\n{}".format(stderr or "", stdout or ""), ("bind failed",))


def _parse_pmc(text):
    data = {"port_states": []}
    offset_text = None
    for key, value in _key_value_lines(text):
        if key == "master_offset":
            offset_text = value
            data["master_offset_ns"] = _to_int(value)
        elif key == "gmPresent":
            data["gm_present"] = value.lower() == "true"
        elif key == "gmIdentity":
            data["gm_identity"] = value
        elif key == "portState":
            data["port_states"].append(value)
    if not data["port_states"] and "master_offset" not in text:
        return None
    if data["port_states"]:
        data["port_state"] = _preferred_port_state(data["port_states"])
    if data.get("port_state") == "SLAVE" and offset_text is not None:
        data["master_offset_ns"] = _required("ptp master_offset", offset_text, _to_int)
    return data


def chrony_status(runner=run_command, config=None):
    _ = config
    answer = _call(runner, ("chronyc", "-c", "tracking"))
    if answer is None:
        return observation(STATE_ABSENT, detail="chronyc is not installed")
    code, stdout, stderr = answer
    if code != 0:
        return _failure(code, stdout, stderr, ABSENT_MARKERS)
    try:
        data = _parse_chrony(stdout)
    except _UnreadableFieldError as error:
        return observation(STATE_UNPARSEABLE, detail=str(error))
    if data is None:
        return observation(
            STATE_UNPARSEABLE, detail=_first_line(stdout) or "chronyc printed no tracking line"
        )
    return observation(STATE_OK, data)


def _parse_chrony(text):
    for line in text.splitlines():
        fields = line.strip().split(",")
        if len(fields) < 14:
            continue
        return {
            "reference_id": fields[0],
            "reference_name": fields[1],
            "stratum": _to_int(fields[2]),
            "offset_s": _required("chrony offset", fields[4], _to_float),
            "root_dispersion_s": _to_float(fields[11]),
            "leap_status": fields[13],
        }
    return None


def timesyncd_status(runner=run_command, config=None):
    _ = config
    answer = _call(runner, ("timedatectl", "show-timesync"))
    if answer is None:
        return observation(STATE_ABSENT, detail="timedatectl is not installed")
    code, stdout, stderr = answer
    if code != 0:
        return _failure(code, stdout, stderr, ABSENT_MARKERS)
    data = {}
    for key, value in _key_value_lines(stdout):
        if key == "ServerName":
            data["server"] = value
        elif key == "ServerAddress":
            data["server_address"] = value
        elif key == "NTPMessage":
            data.update(_parse_ntp_message(value))
    if not data:
        return observation(
            STATE_UNPARSEABLE, detail=_first_line(stdout) or "timedatectl printed no server"
        )
    data["synchronized"] = _ntp_synchronized(runner)
    return observation(STATE_OK, data)


def _parse_ntp_message(value):
    fields = {}
    for item in value.strip("{} ").split(","):
        key, separator, text = item.partition("=")
        if not separator:
            continue
        key, text = key.strip(), text.strip()
        if key == "Stratum":
            fields["stratum"] = _to_int(text)
        elif key == "Leap":
            fields["leap"] = _to_int(text)
        elif key == "Jitter":
            fields["jitter"] = text
    return fields


def _ntp_synchronized(runner):
    answer = _call(runner, ("timedatectl", "show"))
    if answer is None:
        return None
    code, stdout, _stderr = answer
    if code != 0:
        return None
    for key, value in _key_value_lines(stdout):
        if key == "NTPSynchronized":
            return value.lower() == "yes"
    return None


def poll(runner=None, config=None):
    runner = runner or run_command
    sources = {
        "ptp": ptp_status(runner, config),
        "chrony": chrony_status(runner, config),
    }
    answered = sources["chrony"].get("state") == STATE_OK
    sources["timesyncd"] = None if answered else timesyncd_status(runner, config)
    return sources


def _worst(problems):
    return max(problems, key=lambda problem: problem[0])


def _assess_ptp(data, thresholds, context=None):
    offset = data.get("master_offset_ns")
    state = data.get("port_state", "UNKNOWN")
    discipline = (context or {}).get("system_clock")
    values = {
        "port_state": state,
        "gm_present": str(bool(data.get("gm_present"))).lower(),
        "gm_identity": str(data.get("gm_identity", "")),
        "master_offset_ns": str(offset if offset is not None else ""),
    }
    problems = []
    if state in SERVING_PORT_STATES:
        values["system_clock"] = discipline or "none"
        if discipline is None:
            problems.append(
                (
                    WARN,
                    f"ptp port state is {state}, this host serves the clock "
                    f"while nothing disciplines it, "
                    "no healthy chrony or timesyncd observation",
                )
            )
    if state == "SLAVE" and not data.get("gm_present"):
        problems.append((ERROR, "ptp port is SLAVE while no grandmaster is present"))
    if state not in HEALTHY_PORT_STATES:
        problems.append((WARN, f"ptp port state is {state}, neither SLAVE nor MASTER"))
    if state == "SLAVE" and offset is not None:
        magnitude = abs(offset)
        if magnitude >= thresholds["ptp_offset_error_ns"]:
            problems.append((ERROR, f"ptp offset {offset} ns is above the error threshold"))
        elif magnitude >= thresholds["ptp_offset_warn_ns"]:
            problems.append((WARN, f"ptp offset {offset} ns is above the warn threshold"))
    if problems:
        level, message = _worst(problems)
        return level, "ptp", message, values
    if state == "SLAVE":
        message = "ptp is slaved to {} with offset {} ns".format(
            data.get("gm_identity", "a grandmaster"), offset
        )
    else:
        message = (
            f"ptp port state is {state}, this host serves the clock, disciplined by {discipline}"
        )
    return OK, "ptp", message, values


def _assess_chrony(data, thresholds, context=None):
    _ = context
    offset = data.get("offset_s")
    leap = (data.get("leap_status") or "").strip()
    values = {
        "reference_id": str(data.get("reference_id", "")),
        "reference_name": str(data.get("reference_name", "")),
        "stratum": str(data.get("stratum", "")),
        "offset_s": "" if offset is None else f"{offset:.9f}",
        "leap_status": leap,
    }
    normalized = leap.lower()
    if normalized == CHRONY_UNSYNCHRONIZED_LEAP:
        return ERROR, "chrony", "chrony is not synchronized to any source", values
    if normalized != CHRONY_NORMAL_LEAP:
        return WARN, "chrony", f"chrony leap status is {leap}", values
    if offset is not None and abs(offset) > thresholds["chrony_offset_warn_s"]:
        message = f"chrony system time offset {offset:.6f} s is above the warn threshold"
        return WARN, "chrony", message, values
    message = "chrony follows {} at stratum {} with offset {:.6f} s".format(
        data.get("reference_name") or data.get("reference_id"),
        data.get("stratum"),
        offset or 0.0,
    )
    return OK, "chrony", message, values


def _assess_timesyncd(data, _thresholds, context=None):
    _ = context
    values = {
        "server": str(data.get("server", "")),
        "server_address": str(data.get("server_address", "")),
        "stratum": str(data.get("stratum", "")),
        "jitter": str(data.get("jitter", "")),
        "synchronized": str(data.get("synchronized")).lower(),
    }
    server = data.get("server") or data.get("server_address") or "its server"
    if data.get("synchronized"):
        return OK, "timesyncd", f"systemd-timesyncd is synchronized to {server}", values
    return WARN, "timesyncd", "systemd-timesyncd is not synchronized", values


ASSESSORS = {"ptp": _assess_ptp, "chrony": _assess_chrony, "timesyncd": _assess_timesyncd}

PROBLEM_MESSAGES = {
    STATE_PERMISSION_DENIED: "{} state is not readable, access was denied: {}",
    STATE_FAILED: "{} check failed: {}",
    STATE_UNPARSEABLE: "{} answered something this version does not understand: {}",
}


def _assess_problem(name, source):
    state = source.get("state")
    detail = source.get("detail") or "no detail reported"
    message = PROBLEM_MESSAGES[state].format(name, detail)
    return WARN, name, message, {"state": state, "detail": detail}


def _summary(observations, expect_none):
    values = {}
    for name in SOURCE_ORDER:
        source = (observations or {}).get(name)
        values[name] = source.get("state") if source else "not attempted"
    if expect_none:
        return OK, "none", "no time sources observed, none expected on this host", values
    return WARN, "none", "no time sources observed, the clock is unmonitored", values


def system_clock_discipline(observations, settings):
    """Name of a system clock source graded healthy right now, or None."""
    for name in SYSTEM_CLOCK_SOURCES:
        source = (observations or {}).get(name)
        if not source or source.get("state") != STATE_OK:
            continue
        if ASSESSORS[name](source.get("data") or {}, settings)[0] == OK:
            return name
    return None


def assess(observations, thresholds=None, expect_none=None):
    settings = dict(DEFAULT_THRESHOLDS)
    settings.update(thresholds or {})
    if expect_none is None:
        expect_none = bool(settings.get("expect_none"))
    context = {"system_clock": system_clock_discipline(observations, settings)}
    verdicts = []
    for name in SOURCE_ORDER:
        source = (observations or {}).get(name)
        if not source:
            continue
        state = source.get("state")
        if state == STATE_OK:
            verdicts.append(ASSESSORS[name](source.get("data") or {}, settings, context))
        elif state in PROBLEM_STATES:
            verdicts.append(_assess_problem(name, source))
    if not verdicts:
        return [_summary(observations, expect_none)]
    return verdicts
