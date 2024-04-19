# #!/bin/bash

# Path to the autocompletion script
COMPLETION_SCRIPT="$HOME/.upload_docker_image_to_vehicle_completion.sh"

# Create the autocompletion script if it doesn't exist
if [ ! -f "$COMPLETION_SCRIPT" ]; then
    cat <<'EOF' > "$COMPLETION_SCRIPT"
# Autocompletion function for uploading Docker images
_upload_docker_image_to_vehicle_completion() {
    local cur=${COMP_WORDS[COMP_CWORD]}
    local imgs=$(docker images --format "{{.Repository}}:{{.Tag}}" | tr '\n' ' ')
    COMPREPLY=($(compgen -W "${imgs}" -- $cur))
}

complete -F _upload_docker_image_to_vehicle_completion upload_docker_image_to_vehicle.sh
EOF
    echo "Autocompletion script created at $COMPLETION_SCRIPT"
else
    echo "Autocompletion script already exists at $COMPLETION_SCRIPT"
fi

# Check if .bashrc already sources the autocompletion script
if ! grep -q "source $COMPLETION_SCRIPT" "$HOME/.bashrc"; then
    echo "source $COMPLETION_SCRIPT" >> "$HOME/.bashrc"
    echo "Autocompletion script sourced in .bashrc"
else
    echo "Autocompletion script already sourced in .bashrc"
fi

# Get the absolute directory name of the script 
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

# Check if .bashrc already has the updated PATH
#   Use grep with the pattern properly quoted
if ! grep -q "export PATH=.*$SCRIPT_DIR.*" "$HOME/.bashrc"; then
    echo "export PATH=\"$SCRIPT_DIR:\$PATH\"" >> ~/.bashrc
    echo "Docker upload script path added to .bashrc"
else
    echo "Docker upload script path already in .bashrc"
fi

echo "Vehicle upload script installed successfully."
echo "Please source your .bashrc or restart your terminal to apply changes"
echo "  source ~/.bashrc"
