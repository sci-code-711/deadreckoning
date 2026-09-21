---
name: plan-work
description: Use before writing any code for a new feature, bug fix, or piece of work in this repo — takes an idea from a rough suggestion through design and requirements to a set of human-approved GitHub issues. Triggers on "plan this", "let's plan out X", "break this down", "what tickets do we need for X", "scope this out". Does not implement anything itself, and never pushes issues without explicit approval.
---

# Plan work

Turns a request into approved, right-sized GitHub issues in
`sci-code-711/deadreckoning`, in seven stages. Don't skip or collapse
stages — each approval gate exists because the next stage is more
expensive to redo than the last.

## Stages

1. **Suggest an idea.** If the starting point is a rough goal rather than
   a concrete idea, propose one or a few concrete directions before
   diving in — give the user something specific to react to rather than
   open-ended possibility space.
2. **Elaborate.** Once a direction is picked, dig for requirements and
   constraints: read the relevant code (`deadrec/`, `test/`, `CLAUDE.md`,
   `README.md`), and ask about anything that would change the design
   (performance constraints, compatibility, what's explicitly out of
   scope). Keep asking until the shape of the work is actually clear, not
   just until the first obvious question is answered.
3. **Plan the design.** Call `EnterPlanMode`. Design the overall
   architecture/approach — what changes, where, and why over the
   alternatives. This is a design plan, not a ticket list yet.
4. **Review and approval (design).** Call `ExitPlanMode` to present the
   design for approval. If the user pushes back, revise and re-exit
   rather than proceeding on a design that wasn't actually approved.
5. **Plan the breakdown.** Call `EnterPlanMode` again. Split the approved
   design into tickets sized so a human can review each one fully in one
   sitting — prefer more small issues over a few large ones. Note
   dependencies/sequencing between them. For each ticket, invoke
   `write-ticket` in **draft-only** mode to produce title + body text —
   don't create anything on GitHub yet.
6. **Review and approval (drafts).** Call `ExitPlanMode` again, showing
   every drafted ticket's title and body. This is a separate approval
   from step 4 — a good design can still get sliced into bad tickets.
7. **Push to GitHub.** Only after step 6's approval, invoke `write-ticket`
   in **push** mode for each approved draft. Report back the resulting
   issue numbers — those numbers are how this work gets referenced from
   here on, never an internal id.

## Sizing rule

If a draft ticket's acceptance criteria run long enough that a reviewer
would need to re-read it twice, split it. A flat pile of tightly-coupled
tickets at step 5 is a sign the design from step 3 needs another pass, not
just smaller issues.
