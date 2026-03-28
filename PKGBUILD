# Maintainer: daedalao <daedalao@daedalaomain>
pkgname=djcmd
pkgver=0.2.0
pkgrel=1
pkgdesc="A full-featured terminal DJ application optimized for Arch Linux POWER"
arch=('x86_64' 'i686' 'powerpc' 'ppc64' 'ppc64le')
url="https://github.com/djcmd-project/djcmd"
license=('GPL3')
depends=('alsa-lib' 'ncurses' 'sqlite')
makedepends=('git' 'curl')
source=("git+https://github.com/djcmd-project/djcmd.git")
sha256sums=('SKIP')

build() {
	cd "$srcdir/$pkgname"

	# Download single-header dependencies
	make deps

	# Select the appropriate make target based on architecture
	case "$CARCH" in
		x86_64)
			make x86_64
			;;
		i686)
			make i686
			;;
		powerpc)
			make power
			;;
		*)
			# For ppc64/ppc64le or others, use the default target
			# which currently points to 'power' (G4 tuning)
			# We can add specific 64-bit POWER targets to the Makefile later.
			make
			;;
	esac
}

package() {
	cd "$srcdir/$pkgname"
	
	# Install binary
	install -Dm755 djcmd "$pkgdir/usr/bin/djcmd"
	
	# Install documentation
	install -Dm644 README.md "$pkgdir/usr/share/doc/$pkgname/README.md"
	
	# Install default map if provided in source
	if [ -f "mixtrack_3.map" ]; then
		install -Dm644 mixtrack_3.map "$pkgdir/usr/share/$pkgname/maps/mixtrack_3.map"
	fi
}
