# rosio

rosio reads what a ROS 2 system says about itself in four places and reports where those places
disagree. These are the words the code, the documents and the findings use, each with one meaning.

## The model

**Atlas**:
One deterministic description of a system across all its layers, written to `rosio-out/atlas.json`.
_Avoid_: graph, model file, database

**Layer**:
One source of claims about the system: `declared`, `coded`, `observed` or `recorded`. A layer is a
source, never a pipeline stage.
_Avoid_: stage, view, level

**Declared**:
What the repository says should run: launch files, parameter yaml, urdf and xacro, interface
definitions, pluginlib manifests, start scripts.

**Coded**:
What the C++ and Python sources publish, subscribe, declare, read and broadcast, read without
running them.

**Observed**:
What a running system reports to a short probe and through its process table.
_Avoid_: live, runtime layer

**Recorded**:
What a recording holds: topics, types, offered QoS, counts, and the header stamps of its messages.
_Avoid_: bag layer, data layer

**Profile**:
One system a team actually runs, as an ordered set of entry points with their arguments.
_Avoid_: configuration, launch file, setup

**Entity**:
One thing in the atlas with a stable id: a node, a topic, a parameter, a frame, a file.
_Avoid_: object, element

**Fact**:
What one layer stated about one entity or edge, in one profile, with its confidence and evidence.
_Avoid_: observation, record

**Edge**:
A relation between two entities, keyed by the site that produced it when one pair can relate twice.
_Avoid_: link, connection

**Evidence**:
The file, locator, line and tree a fact came from, carried inline on the fact.
_Avoid_: source reference, citation

## Parameters

**Candidate chain**:
Every value that was set for one parameter in one profile, winner first, each with how and where.
_Avoid_: history, overrides

**Effective value**:
The value of a parameter that wins its candidate chain in a profile.
_Avoid_: current value, real value

**Shadowed**:
A candidate that lost, with the candidate that beat it named.

## Findings

**Drift**:
A disagreement between layers, or inside one layer, inside one profile.
_Avoid_: diff, inconsistency, error

**Finding**:
One reported drift: a rule, a severity, a subject, the line to edit, a sentence to quote and a fix.
_Avoid_: issue, warning, problem

**Rule**:
One check from the committed vocabulary, such as `param.shadowed` or `time.epoch_offset`.

**Waiver**:
A decision, with a reason, that one finding no longer grades a CI gate. It never hides the finding.
_Avoid_: suppression, ignore

## What rosio could not read

**Blind spot**:
A value rosio could not determine, recorded with its raw expression instead of a guess. Stored in
`atlas.unresolved`.
_Avoid_: unknown value, gap

**Unproven**:
A finding that would be a claim about everything in a profile while part of the profile runs code
rosio did not read. It is reported at info and names what was not read.
_Avoid_: weak finding, guess

**Resolution rate**:
Resolved values over resolved plus blind spots, per kind of value. A CI gate.

**Refusal**:
An input a collector could not read, recorded on the entity with a reason code.
_Avoid_: error, failure

**Pass**:
One collector's run over its inputs: completed, refused or not attempted.

## Running systems and recordings

**Probe**:
A short read of a running system through rclpy, once per middleware its processes use.

**Middleware**:
The `RMW_IMPLEMENTATION` a process runs under: a DDS implementation or Zenoh.
_Avoid_: transport, DDS (when Zenoh is meant)

**Stamp check**:
A check over the header stamps of one topic against the time the recorder received them.

**Verdict**:
The few sentences a stamp check run ends with, naming the likely clock fault behind its findings.

## Handing a problem over

**Bundle**:
One archive holding an atlas, its reports, the files its findings cite, the configuration and the
git revision, so someone on another machine can open the same findings on the same lines.
_Avoid_: capsule, snapshot, export
