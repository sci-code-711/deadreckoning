---
name: complete-ticket
description: Use when work on a ticket appears finished and needs to be verified and closed out — checks the implementation against the ticket's acceptance criteria, runs the repo's required checks (ruff check, ruff format --check, pytest), and records completion. Triggers on "close this ticket", "mark this ticket done", "wrap up ticket X", "finish up this ticket", "is this ticket done". Does not plan new work.
---

# Complete ticket

Verifies that a ticket's work is actually done and records that, using the
shared ticket format from the `write-ticket` skill. This skill never marks
a ticket done on trust — it checks.

## Steps

1. **Identify the ticket.** Find the relevant file in `tickets/` (by id or
   by matching the current work to a ticket's title/context). If no ticket
   matches the work being closed out, say so rather than inventing one —
   suggest `plan-work` if the work should have had a ticket and didn't.
2. **Check acceptance criteria.** Go through each item in the ticket's
   Acceptance criteria section against the actual diff/code, not against
   what was intended. If any criterion isn't met, stop here: report what's
   missing and leave the ticket's status as `in-progress`, don't close it.
3. **Run the required checks.** Per `CLAUDE.md`, all three must pass
   before a ticket can be marked done:
   ```bash
   uv run ruff check .
   uv run ruff format --check .
   uv run pytest
   ```
   A ticket whose criteria pass but whose checks fail is not done — fix
   the failure or report it; don't close over a red check.
4. **Record completion.** Invoke the `write-ticket` skill to set the
   ticket's `status` to `done` and append a short note under Notes: what
   changed, which files were touched, and anything a future reader would
   need (e.g. a follow-up that was deliberately left out).
5. **Surface follow-ups separately.** If closing this ticket out surfaced
   more work (a bug noticed in passing, a deferred edge case), don't fold
   it into this ticket's scope after the fact — call out that it exists
   and hand it to `plan-work` to become its own ticket.

## What this skill does not do

It doesn't implement missing functionality to make criteria pass, and it
doesn't relax or reinterpret acceptance criteria to make a partial
implementation fit — a ticket that isn't done stays open.
