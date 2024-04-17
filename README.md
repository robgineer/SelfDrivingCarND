This repo contains the code for projects related to the Self-Driving Car Engineer Nanodegree from 2018.

Most of the implementations are still up to date and can serve as a reference for cross checks.

The repo is a result of several repos merged together using the following handy implementation: https://github.com/newren/git-filter-repo

Howto merge repos without loosing the commit history:
* install: `sudo apt-get install git-filter-repo`
* clone repo to be merged as sub-directory into new repo: `git clone <repo A from git> repo_a`
* in `repo_a`: change history refs within repo (as it was always within subdirectory `my_directory`): `git-filter-repo --to-subdirectory-filter my_directory`
* clone repo that should contain `repo A` : `git clone <repo B from git> repo_b`
* in `repo_b`:
  * `git remote add -f external_repo ../repo_a` (adapt path, here: `repo_a` and `repo_b` are in the same directory)
  * `git merge --allow-unrelated-histories external_repo/master`
  * `git remote rm external_repo`
  * `git push`

All contents from `repo_a` are now included within `repo_b/my_directory`



## Contents

* `traffic lights / signs classifications`, `semantic_segmentation`, `vehicle_detection`, `lane finding`: Perception related. Contains traditional ML methods as well as Deep NNs code
* `pid_control`, `path_planning`, `model_predictive_control`, `behavioral_cloning`: Planning specific projects
* `kalman filters`, `localization_project`: Localization related code
