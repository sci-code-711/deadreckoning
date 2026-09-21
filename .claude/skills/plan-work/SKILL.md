---
name: plan-work
description: Use before writing any code for a new feature, bug fix, or piece of work in this repo — turns a request or idea into one or more right-sized, sequenced tickets with clear acceptance criteria. Triggers on "plan this", "let's plan out X", "break this down", "what tickets do we need for X", "scope this out". Does not implement anything itself.
---

# Plan work

Turns a request into a small set of scoped tickets *before* implementation
starts, using the shared ticket format from the `write-ticket` skill. This
skill only plans — it never edits source files.

## Steps

1. **Understand the request.** Read the relevant parts of the codebase
   (`deadrec/`, `test/`, `CLAUDE.md`, `README.md`) to ground the plan in
   what actually exists, rather than assumptions. If the request is
   ambiguous in a way that would change the ticket breakdown, ask — don't
   guess at scope.
2. **Break it into tickets.** Each ticket should be:
   - Small enough to implement, test, and review as one unit.
   - Independently verifiable — its acceptance criteria can be checked
     against a diff without pulling in other tickets' work.
   - Consistent with this repo's conventions (`uv`, `ruff`, `pytest` — see
     `CLAUDE.md`), so acceptance criteria can reference the standard
     checks (`uv run ruff check .`, `uv run ruff format --check .`,
     `uv run pytest`) rather than inventing bespoke ones.
3. **Sequence them.** Note dependencies between tickets explicitly (in
   each ticket's Context section) so `complete-ticket` and future sessions
   know what has to land first.
4. **Draft each ticket.** For every ticket identified, invoke the
   `write-ticket` skill to create the file in the canonical format — don't
   hand-write ticket files directly from this skill.
5. **Confirm before handing off.** Summarize the ticket set (filenames and
   one-line descriptions) for the user. Do not start implementing any of
   them in this skill — that's a separate, explicit step the user takes
   after reviewing the plan.

## When a plan turns out too big

If the breakdown produces more than roughly 5-6 tickets, or several
tickets keep needing to reference each other's internals, step back and
check whether the request itself should be narrowed before drafting
tickets — a flat pile of tightly coupled tickets is a sign the scope needs
another pass, not just smaller files.
