# Copyright (C) Cross The Road Electronics.  All rights reserved.
# License information can be found in CTRE_LICENSE.txt
# For support and suggestions contact support@ctr-electronics.com or file
# an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases

import os
import re
import shutil

from numpy import empty

# Mapping of templates to target directories
TEMPLATE_DIRS: dict[str, str] = {
    "./automation/java_template/" : "./java",
    "./automation/cpp_template/" : "./cpp",
    "./automation/python_template/": "./python"
}

# List of globs to ignore for deletion
DEL_EXCLUSIONS: list = [
    r'src.(\/*|\\)',
    r'.*.py$'
]


# Copy all of the files in the template directory into
# the corresponding example directory
def update_templates(template_dirs, del_exclusions):
    # iterate over templates and globs
    for template_dir, target_seek_dir in TEMPLATE_DIRS.items():
        for root, dirs, files in os.walk(target_seek_dir):
            # Delete all files that aren't regexed
            for file in files:
                full_path = os.path.join(root, file)
                to_keep = False

                # Check to see if we should keep the file
                for glob in DEL_EXCLUSIONS:
                    regex_search = re.search(glob, full_path)

                    if not to_keep:
                        to_keep = regex_search is not None
                
                if not to_keep:
                    print("Removing", full_path)
                    os.remove(full_path)

            for dir in dirs:
                full_dir_path = os.path.join(root, dir)
                if len(os.listdir(full_dir_path)) == 0:
                    print("Removing", full_dir_path)
                    os.removedirs(full_dir_path)

        # Iterate over the examples, copying the files as necessary
        for dir in os.listdir(target_seek_dir):
            target_dir = os.path.join(target_seek_dir, dir)

            for src_path in os.listdir(template_dir):
                source_path = os.path.join(template_dir, src_path)
                target_path = os.path.join(target_dir, src_path)

                # Copy directories
                try:
                    print("Copying source", source_path, "to", target_path)
                    shutil.copytree(source_path, target_path, dirs_exist_ok=True)
                except:
                    # Copy individual files
                    shutil.copy(source_path, target_path)


if __name__ == "__main__":
    update_templates(TEMPLATE_DIRS, DEL_EXCLUSIONS)
