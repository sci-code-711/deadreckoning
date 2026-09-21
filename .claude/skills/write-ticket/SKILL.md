---
name: write-ticket
description: Subskill that defines the canonical GitHub issue format used as this repo's tickets, and drafts or creates a single issue in sci-code-711/deadreckoning. Invoked by plan-work (to draft, then push, ticket bodies) and do-ticket (to record progress comments) rather than used standalone.
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

Depends on: #NN, #NN (or `None`)

## Acceptance criteria

- Concrete, checkable conditions for "done" — behavior, not tasks.
- Sized so a human can review the whole issue in one sitting (see
  plan-work's sizing guidance).

## Out of scope

Anything adjacent this issue deliberately excludes. Omit this section if
there's nothing to exclude.
```

The `Depends on:` line is required, even when the answer is `None` — it's
how `do-ticket` finds unblocked tickets without guessing. List issue
numbers only, not descriptions; an issue is a dependency only if this one
genuinely can't start before it's closed, not just "related to."

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

To record progress on an existing issue (e.g. status notes from
`do-ticket`), use `mcp__github__add_issue_comment` for a log entry.
Issues are normally closed by merging their PR (a `Closes #NN` line in
the PR body), not by editing the issue directly — only use
`mcp__github__issue_write` with `method: "update"` to close one by hand
when a PR isn't the mechanism (e.g. the ticket turned out to be
not-planned or a duplicate). Don't rewrite a ticket's Context/Acceptance
criteria after the fact — append comments instead, so the issue's history
stays honest.
