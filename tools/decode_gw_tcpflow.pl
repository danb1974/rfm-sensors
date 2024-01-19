#!/usr/bin/perl -w

use strict;

$| =  1;

# Usage sample:
#
# tcpflow -cD port 23 | perl decode_gw_tcpflow.pl

# tcpflow output:
#
# 192.168.001.003.46992-192.168.001.161.00023: 
# 0000: de5b 0196 4010  .[..@.

my($src, $dst);

while (my $line = <STDIN>) {
	chomp($line);

	if ($line =~ m!^(\d{3}\.\d{3}\.\d{3}\.\d{3})\.\d+-(\d{3}\.\d{3}\.\d{3}\.\d{3})\.\d+:!) {
		$src = $1; 
		$dst = $2;

		$src =~ s!\.0+!.!g;
		$dst =~ s!\.0+!.!g;

		next;
	}

	if ($line =~ m!^0000: ((?:[0-9a-f]{2,4} )+)!) {
		my $data = $1;
		$data =~ s! +!!g;
		print "FROM $src TO $dst DATA $data";

		my $header = substr $data, 0, 4;
		if ($header eq "de5b") {
			my @bytes;

			my $size = hex(substr $data, 4, 2);
			for (my $i = 0; $i < $size; $i++) {
				my $byte = hex(substr $data, 6 + 2 * $i, 2);
				push(@bytes, $byte);
			}

			# 0 is type, 1 is id, data starts from 2

			if ($bytes[0] == 0x95) {
				print " INIT";
				print "\n";
			}

			elsif ($bytes[0] == 0x96) {
				print " HB";
				if (scalar(@bytes) == 2) {
					print " reply ", $bytes[1];
				} else {
					print " request";
				}
				print "\n";
			}

			elsif ($bytes[0] == 0x92) {
				print " SENDPACKET TO $bytes[1]";

				# actuator-valve de5500c8
				if (scalar(@bytes) == 6 && $bytes[2] == 0xde) {
					print " actuator-valve";
					print " percent ", $bytes[3];
					print " temperature ", ($bytes[4] * 256 + $bytes[5]) / 10.0;
				}

				# sensor-thermostat 020173
				elsif (scalar(@bytes) == 5 && $bytes[2] == 0x02) {
					print " sensor-thermostat";
					print " on ", $bytes[3];
					print " setpoint ", ($bytes[4] + 100) / 10.0;
				}

				# sensor-light-dimmer
				# TODO
				elsif (scalar(@bytes) == 4 && $bytes[2] == 0x01) {
					print " sensor-light-dimmer";
					print " set-brightness ", $bytes[3];
				}
				elsif (scalar(@bytes) == 3 && $bytes[2] == 0x02) {
					print " sensor-light-dimmer";
					print " send-state";
				}
				elsif (scalar(@bytes) == 5 && $bytes[2] == 0x03) {
					print " sensor-light-dimmer";
					print " set-mode ", $bytes[3];
					print " mode-no-dimmer ", $bytes[4];
				}
				elsif (scalar(@bytes) == 4 && $bytes[2] == 0x04) {
					print " sensor-light-dimmer";
					print " set-led ", $bytes[3];
				}
				elsif (scalar(@bytes) == 5 && $bytes[2] == 0x05) {
					print " sensor-light-dimmer";
					print " set-min-light ", $bytes[3];
					print " timeout ", $bytes[4];
				}

				print "\n";
			}

			elsif ($bytes[0] == 0x93) {
				print " PACKETSENT to $bytes[1]";
				print "\n";
			}

			elsif ($bytes[0] == 0x94) {
				print " RECEIVEPACKET FROM $bytes[1]";

				# presence 42ab4c00005301
				if (scalar(@bytes) == 9 && $bytes[2] == 0x42) {
					print " sensor-presence";
					print " voltage ", ($bytes[3] + 100) * 10 / 1000.0;
					print " light ", $bytes[5] * 256 + $bytes[6];
					print " pir ", $bytes[8];
				}

				# light-dimmer 0255 (on but not 100%) 0200 (off)
				elsif (scalar(@bytes) == 4 && $bytes[2] == 0x02) {
					print " sensor-light-dimmer";
					print " light ", $bytes[3];
				}

				# thermostat 011640102f27f0ec
				elsif (scalar(@bytes) == 10 && $bytes[2] == 0x01) {
					print " sensor-thermostat";
					print " temp ", ($bytes[3] * 256 + $bytes[4]) / 256.0;
					print " humidity ", ($bytes[5] * 256 + $bytes[6]) / 100.0;
					print " pressure ", ($bytes[7] * 256 + $bytes[8]) / 10.0;
					print " voltage ", ($bytes[9] + 100) * 10 / 1000.0;
				}

				# valve 42c7
				elsif (scalar(@bytes) == 4 && $bytes[2] == 0x42) {
					print " actuator-valve";
					print " voltage ", ($bytes[3] + 100) * 10 / 1000.0;
				}

				print "\n";
			}

			elsif ($bytes[0] == 0x72) {
				print " ERR_BUSY to $bytes[1]";
				print "\n";
			}

			elsif ($bytes[0] == 0x75) {
				print " ERR_TIMEOUT to $bytes[1]";
				print "\n";
			}

			else {
				print "\n";
			}
		}

		else {
			print "Bad header, probably got more than 1 packet and we do not parse the data properly\n";
		}

		#print "\n";
	}
}
