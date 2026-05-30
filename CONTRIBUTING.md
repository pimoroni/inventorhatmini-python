# Contributing

## Development environment

A Docker-based testing image is provided so checks can be run against multiple Python versions
without installing tooling locally. Python 3.11 is the first version set up; others will follow.

### Build the image

Pass your host UID and GID so that files written inside the container are owned by your user,
not root:

```bash
docker build -f Dockerfile.testing \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  -t inventorhatmini-dev:python3.11-v1.0.1 .
```

> **Image tag convention:** `inventorhatmini-dev:python<python-ver>-v<testing-ver>`
> The testing version is always at least one patch ahead of the released library version
> (e.g. library `1.0.0` → testing image `v1.0.1`).

### Run checks

All commands below mount the repository into the container so changes are picked up without
a rebuild. Run them from the repository root.

**Integrity checks** (trailing whitespace, DOS line-endings, CHANGELOG entry, git tag):

```bash
docker run --rm -v "$(pwd)":/app inventorhatmini-dev:python3.11-v1.0.1 make check
```

**Shell script linting:**

```bash
docker run --rm -v "$(pwd)":/app inventorhatmini-dev:python3.11-v1.0.1 make shellcheck
```

**QA** (ruff, isort, codespell, check-manifest, build, twine check):

```bash
docker run --rm -v "$(pwd)":/app inventorhatmini-dev:python3.11-v1.0.1 make qa
```

### Dependency lock file

The image installs from `requirements-dev.lock`. Regenerate it when `requirements-dev.txt` changes:

```bash
uv pip compile requirements-dev.txt --output-file requirements-dev.lock --python-version 3.11
```

Then rebuild the image.
