# Git Conventions & Workflow

This document defines the exact git workflow to follow on this project.
It is written to be followed by an agent or a human contributor without ambiguity.
Do not deviate from these rules.

---

## Branch Strategy

### `main`
- Always stable and demo-ready.
- Never commit directly to `main`.
- Only updated via merge from a completed feature branch.

### Feature Branches
- Created before any major change begins.
- Named using the pattern:

  ```
  feature/<short-description>
  ```

  Examples:
  ```
  feature/serial-reader
  feature/svd-fitter
  feature/kalman-filter
  feature/visualizer
  ```

- One branch per major change (phase or significant subsystem).
- Branch from the current tip of `main`:

  ```
  git checkout main
  git pull origin main
  git checkout -b feature/<short-description>
  ```

---

## Definitions

| Term | Meaning |
|------|---------|
| **Major change** | A full phase or a self-contained subsystem (e.g., implementing `kalman_filter.py` and `predictor.py`) |
| **Minor change** | A single logical unit of work within a major change (e.g., implementing `predict()`, or adding a unit test for `predict()`) |

---

## Workflow — Step by Step

### 1. Starting a Major Change

```
git checkout main
git pull origin main
git checkout -b feature/<short-description>
```

Document the intent of the branch in the first commit message (see commit format below).

---

### 2. During a Major Change — Minor Change Cycle

Each minor change follows this exact cycle. Do not skip or reorder steps.

```
Step 1 — Implement the minor change
Step 2 — Commit the implementation
Step 3 — Run tests
Step 4 — Document the test results
Step 5 — Commit the test documentation
```

#### Step 2 — Implementation Commit

```
git add <changed files>
git commit -m "<type>(<scope>): <short description>

<body: what was done and why, if not obvious>"
```

#### Step 3 — Run Tests

Run the relevant unit tests for the module just changed.
If no automated test exists for this change yet, write one before proceeding.

For the PC application:
```
python -m pytest tests/ -v
```

For firmware changes, test manually via the OpenMV IDE serial console and note results.

#### Step 4 — Document Test Results

Update or create the relevant test result record.
- For automated tests: capture the `pytest` output.
- For manual firmware tests: note the result in `docs/test_log.md` with the format:

  ```
  ### <date> — <what was tested>
  - Result: PASS / FAIL
  - Notes: <any relevant observations>
  ```

#### Step 5 — Test Documentation Commit

```
git add docs/test_log.md   # or whichever doc was updated
git commit -m "test(<scope>): document results for <what was tested>"
```

---

### 3. Ending a Major Change — Merge to Main

When all minor changes for a major change are complete:

#### 3a. Run the full test suite
```
python -m pytest tests/ -v
```
All tests must pass. Fix any failures before proceeding.

#### 3b. Verify the merge will be clean
```
git checkout main
git pull origin main
git checkout feature/<short-description>
git merge main --no-commit --no-ff
```
Resolve any conflicts now, on the feature branch, not on `main`.
If a conflict was resolved, commit the resolution:
```
git commit -m "chore: resolve merge conflicts with main"
```
Then abort the test merge and return to normal:
```
git merge --abort   # only if --no-commit left it in a pending state
```

#### 3c. Merge into main
```
git checkout main
git merge --no-ff feature/<short-description> -m "feat(<scope>): merge <short-description>"
```
The `--no-ff` flag preserves the branch history. Always use it.

#### 3d. Push main
```
git push origin main
```

#### 3e. Delete the feature branch (local and remote)
```
git branch -d feature/<short-description>
git push origin --delete feature/<short-description>
```

---

## Commit Message Format

All commits must follow this format:

```
<type>(<scope>): <short description>

<optional body>
```

### Types

| Type | Use for |
|------|---------|
| `feat` | New functionality |
| `fix` | Bug fix |
| `refactor` | Code restructure with no behavior change |
| `test` | Adding or updating tests |
| `docs` | Documentation only |
| `chore` | Config, tooling, dependencies, repo structure |
| `perf` | Performance improvement |

### Scope

The scope is the module or subsystem affected. Use the filename without extension or a short label.

Examples: `serial-reader`, `kalman`, `svd`, `visualizer`, `buffer`, `firmware`, `config`, `ci`

### Rules
- Subject line: 72 characters max, lowercase after the type, no period at the end.
- Body (optional): explain *what* and *why*, not *how*. Wrap at 72 characters.
- Do not use `git commit -m` for commits that need a body — open the editor.

### Examples

```
feat(buffer): implement rolling push and NaN sentinel handling
```

```
test(svd): document results for parabolic trajectory recovery test

All three test cases pass. Edge case with fewer than d+2 points
correctly returns None without raising an exception.
```

```
fix(kalman): prevent predict_ahead from mutating live filter state

predict_ahead was operating on self.x directly. Now deep-copies
state and covariance before running lookahead steps.
```

```
chore: initialize repo folder structure per DEVELOPMENT_PLAN.md §3
```

---

## Push Policy

Push to remote in these situations — and only these:

| Trigger | Command |
|---------|---------|
| End of a major change (after merge to main) | `git push origin main` |
| End of a working session | `git push origin <current-branch>` |
| Explicit user request | `git push origin <current-branch>` |

Do not push after every minor change. Pushes are not the same as commits.

---

## Tagging

At demo-ready milestones, create an annotated tag on `main`:

```
git tag -a v1.0-demo -m "Demo-ready release: all phases complete, backup replay recorded"
git push origin v1.0-demo
```

Tag naming convention: `v<major>.<minor>-<label>`

---

## What Not To Do

- Do not commit directly to `main`.
- Do not force-push (`git push --force`) to `main` under any circumstances.
- Do not merge without running the full test suite first.
- Do not skip the test documentation commit step.
- Do not squash the branch history when merging — use `--no-ff`.
- Do not leave a feature branch open after it has been merged.
- Do not commit generated files, virtual environments, or secrets:
  ensure `.gitignore` covers `__pycache__/`, `*.pyc`, `.venv/`, `*.csv` session recordings (unless intentionally committed as demo assets).