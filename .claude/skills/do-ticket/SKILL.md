---
name: do-ticket
description: Use to pick up and implement an open, unblocked GitHub ticket end-to-end — from selecting a ticket through a reviewed implementation plan, the actual implementation, verification, docs, and an open PR that's watched through to merge. Triggers on "what should I work on", "pick a ticket", "let's do #NN", "implement #NN", "work on the next ticket". Does not draft or push new tickets — use plan-work for that.
---

# Do ticket

Takes an approved GitHub issue in `sci-code-711/deadreckoning` from
selection through to a merged PR. Tickets are referenced only by their
GitHub issue number — there's no other id.

## Stages

1. **Find unblocked tickets.** List open issues and filter to ones that
   are actually startable: for each, check its Context section for a
   `Depends on: #NN, ...` line (see `write-ticket`'s ticket format) — an
   issue is unblocked only if every referenced dependency is closed, or
   the line says `None`/is absent. Skip issues already linked to an open
   PR; those are in progress, not unblocked.
2. **Let the user pick.** Present the unblocked candidates (number,
   title, one-line summary of context) and let the user choose which one
   to work on. Don't pick for them, and don't start on one they didn't
   choose.
3. **Plan the implementation.** Call `EnterPlanMode`. Read the chosen
   issue's Acceptance criteria closely and design the implementation
   against the actual codebase — don't just restate the issue as a plan.
   Ask the user to clarify anything the issue leaves ambiguous rather
   than guessing at intent.
4. **Get plan approval.** Call `ExitPlanMode`. If the user pushes back,
   revise and re-exit rather than implementing a plan that wasn't
   actually approved.
5. **Implement.** Make the approved changes. Stay inside the ticket's
   scope — if something out-of-scope surfaces along the way, note it for
   later rather than folding it into this change.
6. **Verify.** Run the repo's required checks and get them clean:
   ```bash
   uv run ruff check .
   uv run ruff format --check .
   uv run pytest
   ```
   Then re-check the diff against the issue's Acceptance criteria one by
   one — a clean test run by itself doesn't mean the criteria are met.
7. **Check docs.** Look for documentation the change makes stale —
   `README.md`, `CLAUDE.md`, docstrings, CLI `--help` text — and update
   it as part of the same change. If nothing needs updating, say so
   explicitly rather than skipping the check silently.
8. **Open the PR.** Push the branch and open a PR whose body includes
   `Closes #NN` so merging it closes the ticket automatically. Follow any
   PR template in the repo. Notify the user that it's open, with the PR
   link.
9. **Watch the PR.** Subscribe to PR activity and follow the repo's
   standing PR-drive rules (fix red CI, address review comments, resolve
   merge conflicts) until it's merged or closed — this is a PR you
   opened, so you own getting it to green, not just reporting its status.

## Anything discovered mid-implementation that's out of scope

Flag it and hand it to `plan-work` to become its own ticket once this one
is out the door — don't silently widen the current PR.

## What this skill does not do

It doesn't draft or push new tickets (that's `plan-work`), and it doesn't
choose a ticket on the user's behalf in step 2 — surfacing candidates and
deferring the choice is the point of that stage.
