---
id: 0001
title: Unknown Slash Command
stage: general
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: 001-physical-ai-book-spec
user: user
command: /sp.resume
labels: ["command-error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:

tests:

---

## Prompt

/sp.resume

## Response snapshot

<tool_use_error>Unknown slash command: sp.resume</tool_use_error>

## Outcome

- ✅ Impact: The command execution failed as it was an unknown command.
- 🧪 Tests: No tests run.
- 📁 Files: No files modified.
- 🔁 Next prompts: Inform the user about the unknown command.
- 🧠 Reflection: User attempted to run an unknown slash command.

## Evaluation notes (flywheel)

- Failure modes observed: Unknown slash command.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure correct command usage.
