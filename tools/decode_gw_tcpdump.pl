#!/usr/bin/perl -w

use strict;
use POSIX qw( strftime );

$| =  1;

# Usage sample, run on the nodered computer:
#
# tcpdump -i eth0 -p -l -s 1500 -x port 23 and host your.ip.address.here | perl decode_gw_tcpdump.pl

sub process_packet {
	my ($data) = @_;

		my $header = substr $data, 0, 4;
		if ($header eq "de5b") {
			my @bytes;

			my $size = hex(substr $data, 4, 2);
			for (my $i = 0; $i < $size; $i++) {
				my $byte = hex(substr $data, 6 + 2 * $i, 2);
				push(@bytes, $byte);
			}

			# 0 is type, 1 is id, data starts from 2

			if ($bytes[0] == 0xff) {
				print " KEEPALIVE";
				print "\n";
			}

			elsif ($bytes[0] == 0x95) {
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
				elsif (scalar(@bytes) == 4 && $bytes[2] == 0x06) {
					print " sensor-light-dimmer";
					print " set-light-curve ", $bytes[3];
				}
				elsif (scalar(@bytes) == 9 && $bytes[2] == 0x03 && $bytes[5] == 0x04 && $bytes[7] == 0x06) {
					print " sensor-light-dimmer";
					print " set-mode ", $bytes[3];
					print " mode-no-dimmer ", $bytes[4];
					print " set-led ", $bytes[6];
					print " set-light-curve ", $bytes[8];
				}

				elsif (scalar(@bytes) == 3) {
					print " one_byte ", $bytes[2];
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

my %buffer;
my %buffer2;
my $bufkey;
my $length;
my($src, $dst);
my $processed = 1;

#19:15:02.776819 IP 192.168.1.136.telnet > 192.168.1.113.42444: Flags [P.], seq 1:8, ack 6, win 1965, length 7
#        0x0000:  4500 002f 0128 0000 8006 b557 c0a8 0188
#        0x0010:  c0a8 0171 0017 a5cc 0000 1a82 fe0b 36ed
#        0x0020:  5018 07ad 20fe 0000 de5b 0296 0080 2c

while (my $line = <STDIN>) {
	chomp($line);
#	print "<<< $line\n";

	if ($line =~ m! IP (\d+\.\d+\.\d+\.\d+).* > (\d+\.\d+\.\d+\.\d+).* length (\d+)!o) {
		$src = $1; 
		$dst = $2;
		$length = $3;
		$processed = 0;
#		print "src $dst dst $dst len $length\n";

		$src =~ s!\.0+!.!g;
		$dst =~ s!\.0+!.!g;
		$bufkey = $src . "-" . $dst;

		next;
	}

	if ($line =~ m!0x[0-9a-f]{4}: +(.+?)$!io) {
		if ($processed == 1) {
			# extra data line
			next;
		}

		my $data = $1;
		$data =~ s! +!!g;

		$buffer2{$bufkey} .= $data;

		if (length($buffer2{$bufkey}) < (40 + $length) * 2) {
			next;
		}

		# keep only declared payload
		$buffer{$bufkey} = substr $buffer2{$bufkey}, 40 * 2, $length * 2;
		$buffer2{$bufkey} = "";
#		print "processing $buffer{$bufkey}\n";

		$processed = 1;

		while (1) {
			if ($buffer{$bufkey} eq "") {
				last;
			}

			if (substr($buffer{$bufkey}, 0, 4) ne "de5b") {
				$buffer{$bufkey} =~ m!^(.*?)(de5b.*)?$!;
				my $noise = $1;
				$buffer{$bufkey} = $2 || "";

				print "Packet with extra bytes $src to $dst: noise '$noise', keeping '$buffer{$bufkey}'\n";
			
				last;
			}

			my $dataSize = hex(substr $buffer{$bufkey}, 4, 2);
			my $totalSize = 6 + $dataSize * 2 + 4;

			if (length($buffer{$bufkey}) < $totalSize) {
				last;
			}

			#print "SIZE $totalSize ";
			print strftime("%Y-%m-%d %H:%M:%S", localtime time), " ";
			#print time(), " ";
			$data = substr($buffer{$bufkey}, 0, $totalSize, "");

			print "FROM $src TO $dst DATA $data";
			process_packet($data);
		}
	}
}
