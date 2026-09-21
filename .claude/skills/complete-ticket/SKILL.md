---
name: complete-ticket
description: Use when work on a GitHub issue (ticket) appears finished and needs to be verified and closed out — checks the implementation against the issue's acceptance criteria, runs the repo's required checks (ruff check, ruff format --check, pytest), and closes the issue with a completion comment. Triggers on "close #NN", "mark this ticket done", "wrap up ticket #NN", "finish up this ticket", "is #NN done". Does not plan new work.
---

# Complete ticket

Verifies a GitHub issue's work is actually done before closing it — never
closes on trust. Tickets are referenced only by their GitHub issue number
in `sci-code-711/deadreckoning`.

## Steps

1. **Identify the issue.** Get the issue number from the user, the branch
   name, or a commit message (e.g. `Closes #42`). If none is given and
   none is inferable, ask — don't guess which issue this work is for.
2. **Read the issue.** Fetch it and check every item in its Acceptance
   criteria section against the actual diff/code, not against what was
   intended. If any criterion isn't met, stop here: report what's missing
   and leave the issue open.
3. **Run the required checks.** Per `CLAUDE.md`, all three must pass
   before an issue can be closed:
   ```bash
   uv run ruff check .
   uv run ruff format --check .
   uv run pytest
   ```
   Criteria passing alongside a red check is not done — fix the failure
   or report it; don't close over it.
4. **Close it.** Use `write-ticket`'s update path: post a completion
   comment via `mcp__github__add_issue_comment` (what changed, which
   files/PR), then close the issue with `mcp__github__issue_write`
   (`state: "closed"`, `state_reason: "completed"`).
5. **Surface follow-ups separately.** If closing this out revealed more
   work (a bug noticed in passing, a deferred edge case), don't fold it
   into this issue after the fact — flag it and hand it to `plan-work` to
   become its own ticket.

## What this skill does not do

It doesn't implement missing functionality to make criteria pass, and it
doesn't reinterpret acceptance criteria to fit a partial implementation —
an issue that isn't done stays open.
