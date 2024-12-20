# Contributing

We encourage community contributions to this repository. Feel free to open an issue or provide a pull request.

# Licensing

Any contribution to this repository will be under the  BSD  License.

## Tooling

Please make sure for any pull request that it passes the `pre-commit` checks (see Tooling section).


### pre-commit

The [pre-commit](https://pre-commit.com/) tool is used for running certain checks and formatters every time before a commit is done.
Please use it on any pull requests for this repository.

__Installation:__

Ubuntu 24.04: `apt install pipx && pipx install pre-commit`. Open a new terminal afterwards.

__Usage:__
In the repository folder run: `pre-commit run --all`. This can be automated with `pre-commit install`, so all checks are run every time a commit is done.
