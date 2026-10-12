# ESCORT — Q&A

Companion to [`ESCORT.narrative.md`](ESCORT.narrative.md) (what you say) and
[`ESCORT.runcard.md`](ESCORT.runcard.md) (what you type). This is what you say
when someone asks something you did not plan to cover.

**How to use it.** Each entry has a **short answer** you can say standing up,
and sometimes a *push* line for when they want more. Say the short one and
stop. The fastest way to lose a technical room is to answer a one-line
question for ninety seconds.

Two rules that matter more than any answer here:

1. **"I don't know, I can find out" is a winning answer.** This is a research
   rig. Nobody expects it to be finished. Guessing in front of an engineer who
   knows the field is the only way to actually lose credibility.
2. **Never upgrade a claim under pressure.** Everything below is scoped to
   *this room, these four aircraft, these numbers*. If someone pushes you to
   say it generalises, the answer is "that is the next question, not a result
   we have."

---

## If you only get one question

> **"So what am I looking at?"**

Three small drones hold a defensive ring around a larger aircraft that a
person is flying by hand. A fourth drone attacks. The ring rotates on its own
so that whichever defender is nearest the threat ends up between the threat
and the thing it is protecting — and it keeps doing that as the person moves
the aircraft around. Nothing about the attack or the response is scripted;
the only scripted part is the attacker's opening move.

---

## Tier 1 — what almost everyone asks

**"Is this pre-programmed?"**
The attacker's approach is, the defence is not. The defenders are solving for
where the threat is right now, twenty times a second. Move the protected
aircraft and the whole ring follows and re-aims — that is the part you cannot
fake.
*Push:* four runs that differed only in where the defenders started produced
355° of spread in the attacker's path. Same code, same settings, completely
different encounters.

**"Who is flying what?"**
The big one is hand-flown, by a person, deliberately. The four small ones are
autonomous — nobody is holding a controller for them.

**"What happens if they crash into each other?"**
They have minimum separations enforced in software on every single command
before it is sent, so a command that would put two drones too close is
modified before it leaves the laptop. There is also a geofence, automatic
landing if the cameras lose an aircraft, and a one-click emergency stop.
*Push:* the floors are 0.8 m between defenders and 1.0 m
defender-to-protected-aircraft. The attacker keeps itself 0.8 m from every
defender; a defender only gives way if the attacker gets inside 0.7 m.

**"How does it know where everything is?"**
Motion capture. The cameras around the room track reflective markers and
stream positions at 50 Hz. All the decisions happen on this laptop and go out
over a single radio.

**"How fast are they going?"**
Everything is capped at 0.6 m/s, the attacker included — slower than
walking pace. It looks faster than it is because the room is small.

**"Why is that one a different colour?"**
Colours are status, not decoration — role and stage. Red is the attacker,
white is whichever defender is currently blocking, violet the other two, deep
blue a ring with nothing to do. Cyan means the ring has parked itself and is
waiting for the pilot. (Full table in the narrative.)

**"How long did this take?"**
Be honest and specific — it lands better than a round number, and it is the
question supervisors actually care about.

---

## Tier 2 — engineers and technical peers

**"What's the control loop?"**
Positions in at 50 Hz from mocap, full-state setpoints out at 20 Hz per
aircraft over one radio. Onboard, each drone runs its own estimator and
attitude loop; we are commanding position, not attitude.

**"Is the attacker a planner or a potential field?"**
Potential field — attraction to the target, repulsion from each defender, plus
a gap-seeking term that biases it toward the widest opening in the ring, and a
give-up timer. It is deliberately not a planner: we want it to look like it is
improvising, and we want it to be beatable in a way we can reason about.

**"How does the ring decide who blocks?"**
It computes the bearing from the protected aircraft to the threat and rotates
the whole formation so a defender lands on that bearing. Slots are assigned by
their bearing around the protected aircraft, fixed once at the start, so
they never swap places mid-run. Which defender blocks is simply whichever slot
is already nearest the threat bearing, so the ring never turns more than 60°.

**"Three defenders can't cover a circle."**
Correct, and that is the honest framing: at a 1.0 m ring the gap between
adjacent defenders is 1.73 m, which a drone fits through easily. The ring
doesn't cover the circle — it *rotates to meet the threat*. Covering bearings
the attacker is not using is measurably worse; we tried spreading them out and
the attacker got closer, not further away.

**"So could it get through?"**
Yes, if it is fast enough, and we know roughly where the line is. The race is
angular, not linear — the defenders are on the inside track, so at the same
linear speed they win on angular rate. Break-even for this geometry is about
0.99 m/s of attacker speed; it flies at 0.6 at most. That number is a calculation
we have partially validated, not a measured wall.

**"What's your latency?"**
Radio round-trip measured 28–87 ms per aircraft this morning. The control
bandwidth is nowhere near that limit — this is a slow position task.

**"What happens if you lose tracking on one?"**
It holds its last commanded point for 0.3 s, and lands itself if nothing comes
back within 1.0 s. If the *protected* aircraft goes dark, the ring parks where
it is rather than chasing a guess. The drones' own estimators are force-fused
to mocap, so without mocap they drift — holding and landing is the only safe
response, not continuing.

**"Why one radio? Isn't that a bottleneck?"**
It is, and it is the most interesting engineering problem in the rig. Sharing
one 2.4 GHz link across four aircraft plus a 50 Hz broadcast means airtime is
the scarce resource; we have had telemetry failures that traced to a drone
going one second without hearing from us, which makes its firmware drop its
own logging. There is now an automatic recovery for that. Happy to go into it
if you want the war story.

**"Simulation first, or straight to hardware?"**
Simulation first, always, and there is a planner that re-verifies the geometry
against the measured room before anything arms — separations, the arena
envelope, altitudes. It refuses to fly a plan that does not fit. It has
refused, correctly, more than once.

---

## Tier 3 — defence / counter-UAS visitors

These are the people most likely to probe the gap between demo and capability.
Lead with the limit, then the value. It reads as competence, not weakness.

**"Would this work outdoors?"**
Not as it stands — it depends entirely on motion capture, so it is an indoor
rig. Replacing that with onboard sensing is a different and much harder
project. What transfers is the behaviour layer: the slot assignment, the
rotate-to-threat logic, the separation guarantees.

**"GPS-denied? Jamming?"**
Out of scope here, and I would not want to imply otherwise. Everything you are
seeing assumes perfect position knowledge and a clean radio link. The
interesting question is what degrades first, and we have partial answers
because we have broken the radio link accidentally.

**"Does it scale past three defenders?"**
Not in this room — physically. Six or more aircraft do not fit the tracked
volume with these separations. The logic is not written around three; the room
is the constraint.

**"Would a net / jammer / shotgun not be simpler?"**
Usually yes, and that is a fair challenge. The thing being demonstrated is not
a better interceptor — it is coordinated autonomous positioning against a
target that is moving unpredictably, which is the part that generalises.

**"Do the defenders physically stop it?"**
No, and this is important: they hold their line and the attacker gives way.
Collision avoidance is assigned to the attacker (0.8 m), which is exactly
what lets the defenders hold position instead of backing off; a defender only
gives way as a last resort, inside 0.7 m. Nobody rams anybody.

**"What's the detection range?"**
There is no detection — the position comes from mocap, so it is given, not
sensed. Engagement starts at 2.10 m, but that is a threshold in software, not
a sensor horizon.

---

## Tier 4 — skeptical and awkward

**"This is just a toy demo though, isn't it?"**
The aircraft are toys, more or less. The coordination problem is not, and it
is the part we are actually working on. Small cheap airframes are how you get
to run the experiment a hundred times without writing anything off.

**"What if I walk into the middle of it?"**
Please don't — but if you do, there is an emergency stop that cuts every motor
instantly, and it is one click, and it is tested. Standing behind the line is
the real answer.

**"What happens if you switch the cameras off?"**
Everything lands. That is designed, not lucky: no position, no flight.

**"Can I fly the attacker?"**
There is a mode for exactly that — a person drives the attacker with a
keyboard and the defence reacts live. It is more convincing than the scripted
version and it is the direction this is going. Not today, because it has not
had enough hours on it to put in front of guests.

**"Did you write all this or is it off the shelf?"**
Be specific and generous: the flight stack underneath is open source
(Crazyswarm2 / Crazyflie), the behaviour, safety layer and choreography are
ours. Claiming the whole stack is the fastest way to get caught.

**"It looked like it got through."**
If it did, say so immediately — see the next section.

---

## Supervisors and management

**"Is it ready to show externally?"**
Give them a scoped yes: it is ready as a demonstration with a trained operator
in a prepared room. It is not a product and it is not unattended.

**"What's the risk of it going wrong in front of a visitor?"**
Non-zero, and here is what "wrong" looks like: a drone lands itself, or the
ring parks and waits. Both are the safety layer working, both are explainable
in one sentence, and neither involves a crash. The failure mode we have
actually seen most is the *protected aircraft* being flown out of the box.

**"What's next?"**
Stress test and rehearse. Then the human-driven attacker, which is what makes
it a demonstration of autonomy rather than a loop.

**"What would you need to go further?"**
Answer with the real blocker, not a wish list. The honest one is room: the
arena limits both the number of aircraft and the speeds, and the walking-VIP
case does not clear its separation margins in simulation at any speed
against the reactive attacker, which is why the pilot brief is to hold the
protected aircraft still while it is under attack.

---

## When it goes wrong, live

Have these ready as sentences. Narrating a failure confidently is better than
any successful run.

| what happened | what you say |
|---|---|
| **the attacker reaches the aircraft** | "That one got through — and that is the honest state of it. Three defenders cannot seal a circle; they can only rotate to meet the threat, and if the aircraft is moving at the same time they run out of time. We tightened exactly this yesterday." |
| **the ring goes cyan and stops** | "The geofence just tripped. The aircraft went outside the box the ring can hold, so the ring parked itself rather than follow it out. I fly back toward the middle and they pick it up again." |
| **a drone lands by itself** | "It lost tracking, so it landed. That is the rule: no position, no flight. Nothing to fix, we carry on with three." |
| **a drone sits still / never arms** | "Firmware safety latch. It takes a battery cycle to clear, which is deliberate. We fly the demo without it." |
| **E-STOP fired** | "That was me, or the software. Everything on the floor is the correct outcome of an abort — the point is that it is one click and it always works." |
| **nothing happens at all** | "Give me a moment — this is a research rig, not an appliance." Then stop talking and fix it. Do not debug out loud. |

---

## Hard limits — the honest list

Keep these straight in your head. Every one of them will eventually be asked,
and every one of them is a better answer given freely than extracted.

* **Indoor only.** Total dependence on motion capture.
* **No sensing.** Positions are given to the system, not perceived.
* **Three defenders do not seal a ring.** 1.73 m gaps; it rotates instead.
* **The attacker yields, not the defenders.** Avoidance is assigned to the
  attacker on purpose (0.8 m); defenders give way only inside 0.7 m.
* **The attacker can win if it is fast enough.** ~0.99 m/s break-even for this
  geometry; it flies at 0.6 at most (its planner runs at 0.8, but its
  commands go through the same 0.6 m/s guard as a defender's).
* **The walking case does not clear.** Against the reactive attacker, any
  motion of the protected aircraft during the attack takes a defender inside
  0.75 m of the attacker in simulation (the scripted attacker clears
  0.15 m/s). Pilot brief: hold still while it is under attack.
* **Four aircraft, one radio.** Six-plus does not fit this room.
* **The protected aircraft has a 0.90 m box.** Smaller than people expect, and
  the most likely thing to interrupt a run.
* **This tuning is new.** The current engagement distance was measured in
  simulation and has had very little hardware time.

---

## Numbers card

Read off this rather than from memory — prose copies of these have drifted
five separate times in this repo.

```
defenders        3, capped 0.6 m/s, ring 1.0 m radius, 1.2 m up
                 (+0.30 m above the protected aircraft when it is flying)
attacker         1, reactive; plans at 0.8 m/s, FLIES at 0.6 m/s max
                 (same guard as a defender) -- the race is angular
engagement       starts at 2.10 m, releases at 2.22 m
separation       0.8 m defender-defender, 1.0 m defender-to-protected,
floors           attacker keeps 0.8 m off defenders, defenders yield at 0.7 m
slot order       by bearing, fixed at the gather; nearest slot blocks
arena            2.00 m tested radius, 1.90 m planned, ceiling 1.85 m
pilot box        0.90 m from room centre; resumes inside 0.65 m
mocap            50 Hz   setpoints 20 Hz per aircraft   one radio
lost tracking    hold 0.3 s, land 1.0 s (0.4 / 2.0 s for the protected one)
ring gap         1.73 m between adjacent defenders
break-even       ~0.99 m/s attacker speed
walking VIP      not cleared with the reactive attacker -- hold still
```

Anything not on this card: `ros2 run crazyflie_shows plan_escort --marks`, or
the banner `escort_show` prints before it arms. Those read the live config.
