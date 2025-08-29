#!/bin/zsh

tag=$(git describe --tags --abbrev=0)
major=$(echo $tag | cut -d. -f1)
minor=$(echo $tag | cut -d. -f2)
patch=$(echo $tag | cut -d. -f3)
new_patch=$((patch + 1))
new_tag="$major.$minor.$new_patch"

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

ssh jenkins.admin.frm2.tum.de -p 29417 build docker/MLZDebianPackage -s -p PROJECT=jcns/tango/franka -p BASEIMAGE=mlzbase/trixie -p BUILDFROM=build -p BRANCH=master > /dev/null 2>&1 &
ssh jenkins.admin.frm2.tum.de -p 29417 build docker/MLZDebianPackage -s -p PROJECT=jcns/tango/franka -p BASEIMAGE=mlzbase/bookworm -p BUILDFROM=build -p BRANCH=master > /dev/null 2>&1 &
#ssh jenkins.admin.frm2.tum.de -p 29417 build docker/MLZDebianPackage -s -p PROJECT=jcns/tango/franka -p BASEIMAGE=mlzbase/noble -p BUILDFROM=build -p BRANCH=master > /dev/null 2>&1 &
#ssh jenkins.admin.frm2.tum.de -p 29417 build docker/MLZDebianPackage -s -p PROJECT=jcns/tango/franka -p BASEIMAGE=mlzbase/jammy -p BUILDFROM=build -p BRANCH=master > /dev/null 2>&1 &
