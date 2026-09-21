---
name: write-ticket
description: Subskill that defines the canonical GitHub issue format used as this repo's tickets, and drafts or creates a single issue in sci-code-711/deadreckoning. Invoked by plan-work (to draft, then push, ticket bodies) and complete-ticket (to close an issue with a completion comment) rather than used standalone.
---

# Write ticket

Tickets in this repo *are* GitHub issues in `sci-code-711/deadreckoning` —
there is no separate file format. Tickets are referenced only by their
GitHub issue number (`#42`), never by an internal id.

## Ticket format

Title: short, specific summary of the change — same bar as a good commit
subject.

Body:

```markdown
## Context

Why this exists and any background needed to act on it without re-reading
the planning conversation. Link related issues with `#NN`.

## Acceptance criteria

- Concrete, checkable conditions for "done" — behavior, not tasks.
- Sized so a human can review the whole issue in one sitting (see
  plan-work's sizing guidance).

## Out of scope

Anything adjacent this issue deliberately excludes. Omit this section if
there's nothing to exclude.
```

## Drafting vs. pushing

This skill has two distinct calls, and callers must say which they want:

- **Draft only** — produce the title + body text above for review. Do
  NOT call `mcp__github__issue_write` yet. This is what `plan-work` uses
  before human approval.
- **Push** — after a draft has been explicitly approved, create it with
  `mcp__github__issue_write` (`method: "create"`, `owner: "sci-code-711"`,
  `repo: "deadreckoning"`). Report back the resulting issue number and
  URL — that number is now the ticket's only identifier.

Never push a ticket that wasn't shown to the user in draft form first.

## Updating an existing ticket

To record progress or completion on an existing issue (e.g. from
`complete-ticket`), use `mcp__github__add_issue_comment` for a log entry
and `mcp__github__issue_write` with `method: "update"` for state changes
(e.g. closing it). Don't rewrite a ticket's Context/Acceptance criteria
after the fact — append comments instead, so the issue's history stays
honest.
