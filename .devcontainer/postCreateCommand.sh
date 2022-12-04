
#!/bin/bash

# Install utilities
packages=(
    fzf
    bash-completion
)

sudo apt-get update
DEBIAN_FRONTEND=noninteractive sudo apt-get install -y --no-install-recommends "${packages[@]}"