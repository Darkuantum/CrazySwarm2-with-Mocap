# ESCORT — what to say to guests

A presenter's script, mapped to the operator's Enter presses. The show is
operator-paced, so **every stage can be held for as long as the talking
needs** — nothing advances until someone presses Enter. Roughly 4-6 minutes
with pauses.

Two people: one on the console (presses Enter, can also narrate), one flying
the DJI. Agree beforehand who says "bring it up" and "bring it down", because
those two moments are the only ones where the script waits on the pilot
rather than on the operator.

Run order and commands: [ESCORT.runcard.md](ESCORT.runcard.md).

---

## Before anything flies (~45 s)

> "What you're about to see is a protection problem. This DJI is standing in
> for something we want to keep safe — a person, a vehicle, a piece of
> equipment. These three small drones are its escort. This fourth one is
> going to try to reach it.
>
> Two things make this harder than it looks. None of these drones can see
> each other — there are no cameras or sensors on board for that. Everything
> they know comes from the motion-capture cameras around the room, which tell
> a computer where everything is, fifty times a second. And the DJI is
> **hand-flown**, so nothing here is a rehearsed routine. The escort has to be
> worked out live, as it happens."

## 1 — the escort takes off

*Enter. Four Crazyflies lift; the three defenders climb to 1.7 m.*

> "The escort goes up first, and deliberately sits **high** — above where the
> DJI will be. They stay out of the way until it is safely in position."

## 2 — the DJI comes up into the ring

*Ask the pilot to bring the DJI up. Enter once it is settled.*

> "Now they drop down and lock on. From this moment the formation belongs to
> the DJI — wherever the pilot flies it, the ring goes with it, holding a
> metre out and a little above. Watch what happens when he moves."

*Good place to pause and let the pilot drift around. This is the clearest
demonstration that nothing is pre-planned.*

## 3 — the attacker takes station

*Enter.*

> "Our fourth drone is the threat. It is coming round to one side to size up
> the formation. It is **amber** right now — it has not committed to
> anything yet."

## 4 — the attack

*Enter. The centrepiece. Let it run.*

> "Now it is red, and it is trying to reach the DJI.
>
> Here is the part worth watching: that drone is **not** following a script.
> Every twentieth of a second it looks at where the three defenders actually
> are, finds the widest gap in their formation, and goes for it. Run this ten
> times and you will get ten different approaches.
>
> And the defenders are reacting, not replaying. Whichever one is best placed
> becomes the blocker — it turns **white** — and the formation rotates to
> keep it between the attacker and the DJI. The other two close up alongside.
>
> The counter-intuitive bit: the attacker is *faster* than the defenders. It
> still cannot get through, because the defenders are flying a tighter circle
> — they cover the same angle in less distance. They win on geometry, not on
> speed."

## 5 — it stands down

*Enter.*

> "It has run out of options and it is backing off — that is the magenta. It
> gives up on its own; nobody tells it to."

## 6-7 — bring it home

*Enter: the ring climbs clear. Pilot lands the DJI. Enter: defenders land.*

> "The escort climbs out of the way and holds there while the DJI comes down.
> They land last, so the thing we are protecting is never the last one in the
> air."

---

## Light legend (point at these)

| colour | meaning |
|---|---|
| **red** | the attacker, pressing |
| **amber** | the attacker taking station — or, on ALL drones, the show is waiting on a human |
| **magenta** | the attacker has stood down |
| **white** | the defender currently blocking |
| **violet** | the other two defenders, closed up beside the blocker |
| **deep blue** | the resting ring, nothing to do |
| **cyan** | all three defenders: the geofence has tripped, the ring is parked and waiting for the DJI to come back |

A fleet that has gone uniformly amber means "your move" — the operator or the
pilot owes it an action.

Cyan is the one you most want to recognise, because it is the only cue that
means the show has stopped doing what the script says. The defenders go cyan
*together*, and the attacker keeps its own colour throughout — so "the ring
turned cyan" is unambiguous. Fly the DJI back towards the middle of the arena
and they resume on their own.

## If they ask

**"Could it get through?"**
Honestly, that depends on the numbers. In this room, with these three
defenders, no. Give the attacker enough of a speed advantage and it
eventually out-turns them; we know roughly where that line is.

**"What stops a crash?"**
Minimum separations enforced in software on every command, a geofence so the
DJI cannot be flown out of the tracked area, automatic landing if the cameras
lose anything, and a one-click emergency stop.

**"Is the DJI doing anything clever?"**
No — it is flown by hand. That is deliberate: it is what makes the escort a
live problem rather than a rehearsal.

**"Why those colours?"**
They are status, not decoration. See the table above.

**"How does it know where everything is?"**
Motion capture — the cameras around the room track reflective markers and
stream positions at 50 Hz. Everything the drones do is computed from that,
on a laptop, and radioed out.

---

## Claims to keep honest

* **"It cannot get through"** is a claim about THIS room and THESE numbers.
  It is what has been demonstrated; it is not a general result.
* **"Ten different approaches"** is literally true: four runs that differed
  only in the defenders' starting formation produced 355 degrees of spread in
  the attacker's path.
* Do not describe the defenders as physically stopping the attacker by
  contact. They hold their line and the attacker gives way — collision
  avoidance is assigned to the attacker, which is what lets the defenders
  hold position instead of backing off.
