#!/bin/zsh

tag=$(git describe --tags --abbrev=0)
major=$(echo $tag | cut -d. -f1)
minor=$(echo $tag | cut -d. -f2)
patch=$(echo $tag | cut -d. -f3)
new_minor=$((minor + 1))
new_tag="${major}.${new_minor}.0"

sed "s/VERSION ${tag#v}/VERSION ${new_tag#v}/" CMakeLists.txt > CMakeLists.txt.new
mv CMakeLists.txt.new CMakeLists.txt

header="jcnsfranka (${new_tag#v}) unstable; urgency=medium\n"
changelog=$(git log --pretty=format:"  * %s" ${tag}..HEAD)
sign="\n -- $(git config user.name) <$(git config user.email)> $(date -R)\n\n"
echo -e "$header\n$changelog\n$sign$(cat debian/changelog)" > debian/changelog.new
mv debian/changelog.new debian/changelog

git add debian/changelog
git add CMakeLists.txt
git commit -m "Update changelog for $new_tag release"
git push origin master
git tag -a -m "" $new_tag
git push --tags origin master
