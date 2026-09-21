---
name: write-ticket
description: Subskill that defines the canonical ticket format for this repo and creates or updates a single ticket file under tickets/. Invoked by the plan-work and complete-ticket skills rather than used standalone, but can also be triggered directly with "write a ticket for X" or "draft a ticket for X".
---

# Write ticket

Defines the one ticket format used across this repo, so `plan-work` and
`complete-ticket` produce consistent, readable tickets instead of each
inventing their own shape.

## Where tickets live

- All tickets are markdown files in `tickets/` at the repo root. Create the
  directory the first time it's needed — don't scaffold it speculatively.
- Filename: `NNN-short-slug.md`, where `NNN` is a zero-padded, sequential
  three-digit id (`001`, `002`, ...) and `short-slug` is a few hyphenated
  words from the title. The id is one higher than the highest existing
  ticket id in `tickets/`, or `001` if the directory is empty or missing.

## Ticket format

Each ticket is a single markdown file with YAML frontmatter followed by a
fixed set of sections:

```markdown
---
id: 003
title: Short, specific summary of the change
status: planned
created: 2026-09-21
---

## Context

Why this ticket exists and any background needed to act on it without
re-reading the whole conversation. Link related tickets by filename.

## Acceptance criteria

- Concrete, checkable conditions for "done" — behavior, not tasks.
- Prefer things that can be verified by reading a diff or running a check
  over vague goals.

## Out of scope

Anything adjacent that this ticket deliberately does not cover, so it
doesn't get silently folded in. Omit this section if there's nothing to
exclude.

## Notes

Append-only log of what happened: decisions made while implementing,
follow-ups spun off, and — on completion — a short summary of what changed
and which files were touched. Omit until there's something to record.
```

`status` is one of `planned`, `in-progress`, or `done`. Only
`complete-ticket` moves a ticket to `done`; only move it earlier
(`planned` → `in-progress`) when work on it actually starts.

## What this skill does

Given a ticket's content (from `plan-work`) or a status change plus a
completion note (from `complete-ticket`):

1. Determine the next ticket id by scanning `tickets/` for the highest
   existing `NNN` prefix, if creating a new ticket.
2. Write or update the file in the format above. Keep acceptance criteria
   specific enough that `complete-ticket` can check them against a diff
   without guessing at intent.
3. Report the ticket's filename back to the caller.

Keep tickets small enough to fit the format above without padding — if the
content doesn't fit in a few sentences per section, it's probably more than
one ticket (send it back to `plan-work` to split).
