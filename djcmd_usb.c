/*
 * djcmd_usb.c -- USB export / eject support for djcmd
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "djcmd_usb.h"

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* ── Internal helpers ───────────────────────────────────────────────── */

/* Recursively create directories (like mkdir -p). */
static void usb_mkdir_p(const char *path)
{
	char tmp[1024];
	strncpy(tmp, path, sizeof(tmp) - 1);
	tmp[sizeof(tmp) - 1] = '\0';
	size_t len = strlen(tmp);
	if (len && tmp[len - 1] == '/')
		tmp[--len] = '\0';
	for (char *p = tmp + 1; *p; p++) {
		if (*p == '/') {
			*p = '\0';
			mkdir(tmp, 0755);
			*p = '/';
		}
	}
	mkdir(tmp, 0755);
}

/*
 * Return 1 if src should be copied to dst (dst absent, or size/mtime differ).
 * Return 0 if dst already matches src (skip).
 */
static int file_needs_copy(const char *src, const char *dst)
{
	struct stat ss, ds;
	if (stat(src, &ss) != 0)
		return 0; /* src missing — nothing to copy */
	if (stat(dst, &ds) != 0)
		return 1; /* dst absent — must copy */
	return (ss.st_size != ds.st_size || ss.st_mtime != ds.st_mtime);
}

/* Copy src to dst.  Returns 0 on success, -1 on error. */
static int copy_file(const char *src, const char *dst)
{
	FILE *in = fopen(src, "rb");
	if (!in)
		return -1;
	FILE *out = fopen(dst, "wb");
	if (!out) {
		fclose(in);
		return -1;
	}

	char buf[65536];
	size_t n;
	int ok = 1;
	while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
		if (fwrite(buf, 1, n, out) != n) {
			ok = 0;
			break;
		}
	}
	fclose(in);
	fclose(out);
	if (!ok) {
		remove(dst);
		return -1;
	}
	return 0;
}

/* ── Public API ─────────────────────────────────────────────────────── */

void usb_get_machine_id(char *out, size_t sz)
{
	out[0] = '\0';

	/* Primary: /etc/machine-id (systemd, present on all modern Linux) */
	FILE *f = fopen("/etc/machine-id", "r");
	if (f) {
		if (fgets(out, (int)sz, f)) {
			out[strcspn(out, "\r\n")] = '\0';
			if (out[0]) {
				fclose(f);
				return;
			}
		}
		fclose(f);
	}

	/* Fallback: ~/.config/djcmd/machine_id */
	const char *home = getenv("HOME");
	if (!home) {
		strncpy(out, "unknown", sz - 1);
		return;
	}

	char path[512];
	snprintf(path, sizeof(path), "%s/.config/djcmd/machine_id", home);

	f = fopen(path, "r");
	if (f) {
		if (fgets(out, (int)sz, f)) {
			out[strcspn(out, "\r\n")] = '\0';
			if (out[0]) {
				fclose(f);
				return;
			}
		}
		fclose(f);
	}

	/* Generate and persist a new ID */
	snprintf(out, sz, "%08lx%08lx%08lx", (unsigned long)time(NULL),
		 (unsigned long)getpid(),
		 (unsigned long)(size_t)out); /* address entropy */

	f = fopen(path, "w");
	if (f) {
		fprintf(f, "%s\n", out);
		fclose(f);
	}
}

int usb_scan(USBDevice *out, int max)
{
	int count = 0;

	FILE *f = fopen("/proc/mounts", "r");
	if (!f)
		return 0;

	char line[1024];
	while (fgets(line, sizeof(line), f) && count < max) {
		char device[256], mountpoint[512], fstype[64];
		if (sscanf(line, "%255s %511s %63s", device, mountpoint,
			   fstype) != 3)
			continue;

		/* Check for djcmd-usb marker */
		char marker[768];
		snprintf(marker, sizeof(marker), "%s/%s", mountpoint,
			 DJCMD_USB_MARKER);
		struct stat st;
		if (stat(marker, &st) != 0)
			continue;

		USBDevice *dev = &out[count];
		memset(dev, 0, sizeof(*dev));
		strncpy(dev->mount_point, mountpoint,
			sizeof(dev->mount_point) - 1);

		/* Try to resolve volume label from /dev/disk/by-label/ */
		DIR *d = opendir("/dev/disk/by-label/");
		if (d) {
			struct dirent *ent;
			while ((ent = readdir(d))) {
				if (ent->d_name[0] == '.')
					continue;

				char link_path[384];
				snprintf(link_path, sizeof(link_path),
					 "/dev/disk/by-label/%s", ent->d_name);

				char resolved_label[256], resolved_dev[256];
				if (realpath(link_path, resolved_label) &&
				    realpath(device, resolved_dev)) {
					if (strcmp(resolved_label,
						   resolved_dev) == 0) {
						strncpy(dev->label, ent->d_name,
							sizeof(dev->label) - 1);
						dev->label[sizeof(dev->label) -
							   1] = '\0';
						break;
					}
				}
			}
			closedir(d);
		}

		/* Fall back to basename of the mount point */
		if (!dev->label[0]) {
			const char *bn = strrchr(mountpoint, '/');
			strncpy(dev->label, bn ? bn + 1 : mountpoint,
				sizeof(dev->label) - 1);
			dev->label[sizeof(dev->label) - 1] = '\0';
		}

		count++;
	}

	fclose(f);
	return count;
}

void usb_crate_read_origin(const char *crate_path, char *out, size_t sz)
{
	out[0] = '\0';
	FILE *f = fopen(crate_path, "r");
	if (!f)
		return;

	char line[256];
	while (fgets(line, sizeof(line), f)) {
		if (line[0] != '#')
			break; /* headers are at the top */
		if (strncmp(line, "# origin: ", 10) == 0) {
			strncpy(out, line + 10, sz - 1);
			out[strcspn(out, "\r\n")] = '\0';
			break;
		}
	}
	fclose(f);
}

int usb_export_crate(const char *local_crate_path, const char *local_crate_name,
		     const char *local_origin, const char *usb_mount,
		     const char *export_name, char *status_out,
		     size_t status_sz)
{
	/* ── 1. Ensure USB directory structure ── */
	char dir[1024];
	snprintf(dir, sizeof(dir), "%s/%s", usb_mount, DJCMD_USB_DIR);
	mkdir(dir, 0755);

	/* Write marker */
	char marker[1024];
	snprintf(marker, sizeof(marker), "%s/%s", usb_mount, DJCMD_USB_MARKER);
	FILE *mf = fopen(marker, "w");
	if (mf)
		fclose(mf);

	snprintf(dir, sizeof(dir), "%s/%s", usb_mount, DJCMD_USB_MUSIC);
	usb_mkdir_p(dir);
	snprintf(dir, sizeof(dir), "%s/%s", usb_mount, DJCMD_USB_CRATES);
	usb_mkdir_p(dir);

	/* ── 2. Open source crate ── */
	FILE *src_crate = fopen(local_crate_path, "r");
	if (!src_crate) {
		snprintf(status_out, status_sz, "Cannot open crate: %s",
			 local_crate_name);
		return -1;
	}

	/* ── 3. Open destination USB crate for writing ── */
	char usb_crate_path[1024];
	snprintf(usb_crate_path, sizeof(usb_crate_path), "%s/%s/%s.crate",
		 usb_mount, DJCMD_USB_CRATES, export_name);

	FILE *dst_crate = fopen(usb_crate_path, "w");
	if (!dst_crate) {
		fclose(src_crate);
		snprintf(status_out, status_sz, "Cannot write USB crate");
		return -1;
	}

	fprintf(dst_crate, "%s\n", DJCMD_CRATE_HDR);
	fprintf(dst_crate, "# origin: %s\n", local_origin ? local_origin : "");

	/* ── 4. Process each track ── */
	int copied = 0, skipped = 0, failed = 0;

	char line[2048];
	while (fgets(line, sizeof(line), src_crate)) {
		line[strcspn(line, "\r\n")] = '\0';
		if (!line[0] || line[0] == '#')
			continue;

		const char *abs_path = line;
		/* Relative path on USB: strip leading '/' */
		const char *rel = abs_path;
		if (rel[0] == '/')
			rel++;

		/* ── Audio file ── */
		char usb_audio[4096];
		snprintf(usb_audio, sizeof(usb_audio), "%s/%s/%s", usb_mount,
			 DJCMD_USB_MUSIC, rel);

		/* Create parent directories */
		char parent[4096];
		strncpy(parent, usb_audio, sizeof(parent) - 1);
		parent[sizeof(parent) - 1] = '\0';
		char *last_slash = strrchr(parent, '/');
		if (last_slash) {
			*last_slash = '\0';
			usb_mkdir_p(parent);
		}

		if (file_needs_copy(abs_path, usb_audio)) {
			if (copy_file(abs_path, usb_audio) == 0)
				copied++;
			else
				failed++;
		} else {
			skipped++;
		}

		/* Write relative path into USB .crate */
		fprintf(dst_crate, "%s/%s\n", DJCMD_USB_MUSIC, rel);

		/* ── Sidecar (next to audio, sidecar_path() compatible) ── */
		char sidecar_src[2056], sidecar_dst[4104];
		snprintf(sidecar_src, sizeof(sidecar_src), "%s.djcmd",
			 abs_path);
		snprintf(sidecar_dst, sizeof(sidecar_dst), "%s.djcmd",
			 usb_audio);

		struct stat ss;
		if (stat(sidecar_src, &ss) == 0 &&
		    file_needs_copy(sidecar_src, sidecar_dst)) {
			copy_file(sidecar_src, sidecar_dst);
		}
	}

	fclose(src_crate);
	fclose(dst_crate);

	snprintf(status_out, status_sz,
		 "Exported '%s': %d copied, %d skipped, %d failed", export_name,
		 copied, skipped, failed);
	return 0;
}

int usb_eject(const char *mount_point)
{
	/* Find block device for this mount point */
	char block_dev[512] = "";
	FILE *f = fopen("/proc/mounts", "r");
	if (f) {
		char line[1024], dev[512], mp[512], rest[256];
		while (fgets(line, sizeof(line), f)) {
			if (sscanf(line, "%511s %511s %255s", dev, mp, rest) ==
				    3 &&
			    strcmp(mp, mount_point) == 0) {
				strncpy(block_dev, dev, sizeof(block_dev) - 1);
				block_dev[sizeof(block_dev) - 1] = '\0';
				break;
			}
		}
		fclose(f);
	}

	/* Unmount */
	char cmd[600];
	snprintf(cmd, sizeof(cmd), "umount \"%s\" 2>/dev/null", mount_point);
	if (system(cmd) != 0)
		return -1;

	/* Power off the device (non-fatal if udisksctl is absent) */
	if (block_dev[0]) {
		snprintf(cmd, sizeof(cmd),
			 "udisksctl power-off -b \"%s\" 2>/dev/null",
			 block_dev);
		system(cmd);
	}

	return 0;
}
