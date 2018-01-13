cat gitlog.txt    | awk '{print $1}' > log1.tmp
cat log1.tmp | git cherry-pick -x --stdin
