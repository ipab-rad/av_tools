_record_rosbag_autocomplete() {
    local cur prev opts
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Directory containing topic lists presets
    topic_list_dir="/opt/ros_ws/config/recording_presets"

    COMPREPLY=()

    # Check if the previous argument was -f or --file
    if [[ "$prev" == "-f" || "$prev" == "--file" ]]; then
        # List only the filenames (not full paths)
        local files=$(ls "$topic_list_dir"/*.txt 2>/dev/null | xargs -n1 basename)

        # If the current input (`cur`) starts with the full path, strip the path
        if [[ "$cur" == "$topic_list_dir/"* ]]; then
            cur="${cur##*/}"  # Remove the path, leaving only the filename part
        fi

        # Autocomplete the filenames
        COMPREPLY=( $(compgen -W "$files" -- "$cur") )

        # If the user selects a file, prepend the full path
        if [[ ${#COMPREPLY[@]} -eq 1 ]]; then
            COMPREPLY[0]="$topic_list_dir/${COMPREPLY[0]}"
        fi
    fi
}

# Register autocomplete function
complete -F _record_rosbag_autocomplete record_rosbag.sh
