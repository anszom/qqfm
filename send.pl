#!/usr/bin/perl
#
# Send commands to the qqfm dongle over the audio channel.
# Usage: ./send.pl [commands] | aplay -f dat
# 
# You need to ensure that the volume level is set to an appropriate level
# (above 90% in my case) to trigger the receiver.
#
# Supported commands:
#	nop             - does nothing (but still blinks led)
#	reset           - soft-reboots the avr
#	i2c:AA:BB       - sends an i2c command to ns741. AA and BB are hexadecimal bytes to send.
#	cfg:INDEX:VALUE - changes device configuration. INDEX and VALUE are decimal
#	                    cfg:0:95000 - set transmitter frequency to 95MHz
#	                    cfg:1:3 - set transmitter power to 3 (max)
#	                    cfg:2:1 - set stereo mode to 1 (on)
#	                    cfg:3:0 - set mute to 0 (not muted)
#	                    cfg:4:1 - set boost to 1 (volume boost enabled)
# 	seq:AA:BB       - reconfigures the initialization i2c sequence at index AA to byte BB.
#	                  AA and BB are hexadecimal.
#
# For further information see qqfm.c

$baud = 3200;
$samples_per_bit = int(48000 / $baud)*2;

sub crc16 {
	my ($string, $poly) = @_;
	my $crc = 0;
	for my $c ( unpack 'C*', $string ) {
		$crc ^= $c;
		for ( 0 .. 7 ) {
			my $carry = $crc & 1;
			$crc >>= 1;
			$crc ^= $poly if $carry;
		}
	}
	return $crc;
}

sub send_string
{
	my $str = $_[0];
	my @str = split("", "UUUUqqfm".$str.pack("s", crc16($str, 0xa001)));

	for(@str) {
		# 2. start bit, data, stop bit
		print map { $_=="1" ? $one : $zero } ("0",  split("",unpack("b8", $_)), 1);
	}
}

sub decode_arg 
{
	# Decode command-line arguments
	my $str = $_[0];
	return "\x03_ABCD" if($str eq "nop");
	return "\x04_ABCD" if($str eq "reset");
	if($str =~ /^i2c:(.*):(.*)$/) {
		return "\x02".pack("H2", $1).pack("H2",$2)."XXX";
	}

	if($str =~ /^cfg:(.*):(.*)$/) {
		return "\x01".pack("C", $1).pack("L",$2);
	}
	
	if($str =~ /^seq:(.*):(.*)$/) {
		return "\x05".pack("H2", $1).pack("H2", $2).pack("C", $gseq++)."\x00\x00";
	}

	if($str =~ /^fseq:(.*):(.*)$/) {
		return "\x05".pack("H2", $1).pack("H2", $2).pack("C", $gseq++)."\xff\x00";
	}

	return pack("H*", $str);
}

for(reverse 0..$#ARGV) {
	last if($ARGV[$_] =~ s/^seq/fseq/);
}

my @cmds = map { decode_arg($_) } @ARGV;

$z = pack("S", 0) x $samples_per_bit;
# try positive polarity
$l0=-32768;
$l1=32767;

$one = pack("S", $l1) x $samples_per_bit;
$zero = pack("S", $l0) x $samples_per_bit;

print $z x 100;
send_string($_) for @cmds;

# now try negative polarity
$l0=32768;
$l1=-32767;
$one = pack("S", $l1) x $samples_per_bit;
$zero = pack("S", $l0) x $samples_per_bit;

print $z x 100;
send_string($_) for @cmds;

print $z x 100;


