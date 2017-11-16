# get all changes in broonie-next since last backport
git log --oneline --reverse --no-merges v4.14-backport-latest..broonie/for-next \
    > log_broonie.tmp

# get log based on directory (can be edited at will)
git log --oneline --reverse --no-merges v4.14-backport-latest..broonie/for-next -- \
    sound/soc/intel/ sound/hda include/uapi/sound/snd_sst*.h      \
    sound/soc/codecs/rt564* sound/soc/codecs/rt565* sound/soc/codecs/rt567* sound/soc/codec/rt28* sound/soc/codec/rt29* \
    sound/soc/codecs/max* sound/soc/codecs/da* sound/soc/codecs/nu* \
    > log_dir.tmp

# get log from sound/ and include/sound based on authors from intel
git log --oneline --reverse --no-merges v4.14-backport-latest..broonie/for-next --author=intel.com -- sound/ include/sound > log_intel.tmp

# merge SHA1s
cat log_intel.tmp log_dir.tmp | sort -u > list.tmp

# extract the relevant SHA1s from the initial list
grep -F -x -f list.tmp log_broonie.tmp > gitlog.txt
