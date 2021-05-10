PACKAGE_NAME="dragon-eye"
VERSION=0.0.1.0

all ::
	@echo "[INFO] Create debian package file (control)."
	SIZE=`du -sx --exclude DEBIAN "./repack/DEBIAN/" | awk '{print $$1}'` ;\
	sed -i "s/Installed-Size: 0/Installed-Size: $${SIZE}/g" "./repack/DEBIAN/control" ;\
	sed -i 's/Version: 0.0.0-0/Version: ${VERSION}/g' "./repack/DEBIAN/control" ;\
	cat ./repack/DEBIAN/control
	@echo "[INFO] Build Deb Package."
	rm -rf "./${PACKAGE_NAME}_${VERSION}.deb"
	dpkg -b "./repack" "./${PACKAGE_NAME}_${VERSION}.deb"

install ::
	dpkg -i "./${PACKAGE_NAME}_${VERSION}.deb"
