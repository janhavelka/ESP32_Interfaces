# Agent instructions

## Git workflow

- Before editing, fetch and inspect the current branch and upstream; fast-forward
  only when safe.
- Keep the whole promptset on its existing branch. Do not create a branch or
  worktree per prompt or agent unless the user asks.
- Assume concurrent agents. Preserve unrelated changes; never stash, discard,
  overwrite, or commit them. A dirty worktree alone is not a blocker.
- After relevant checks pass, stage only your own files or hunks, then commit
  and push each meaningful completed chunk, including multiple times within
  one prompt and before finishing. No further permission is needed.
- When using subagents, the lead owns Git mutations for their shared work.
- Confirm upstream synchronization and check applicable CI for the pushed commit.
  Report conflicts or sync failures; never force-push to resolve them.
