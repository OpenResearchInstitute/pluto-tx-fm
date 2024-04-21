#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <unistd.h>
#include <math.h>
#include "getopt.h"

#define	MAX_CONTEXT_URL_LEN 80
#define MAX_SAMPLE_VALUE	0x7FFF

size_t buffer_size = 0;				// computed from sample_rate if not specified
long long sample_rate = -1;			// command line must specify this
long long center_frequency = -1;	// command line must specify this
long max_deviation = 10000;				// like regular amateur FM modulation
int transmit_attenuation = 10;		// minimum "safe" attenuation (for Pluto loopback)
int xo_correction = -465;			// crystal oscillator correction for my Pluto
char iio_context_url[MAX_CONTEXT_URL_LEN+1] = "ip:pluto.local";	// default value often works
bool status_display = 1;			// default to chatty status display (change with -q)
bool offset_lo = 0;					// default to signal centered on the LO frequency
long long offset_lo_offset = 0;		// frequency offset used with -E flag

double time_per_sample;				// reciprocal of sample_rate
double deviation_scale_factor;		// multiply this by incoming sample to get deviation in Hz

/* Signal generator */
extern void next_tx_sample(int16_t * const i_sample, int16_t * const q_sample);

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define IIO_ENSURE(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog bandwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
	long long tx_gain;	// transmitter gain (negative of attenuation)
	const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
//static struct iio_channel *rx0_i = NULL;
//static struct iio_channel *rx0_q = NULL;

static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
//static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown(void)
{	
	if (status_display) printf("* Destroying buffers\n");
//	if (rxbuf) { iio_buffer_destroy(rxbuf); }
	if (txbuf) { iio_buffer_destroy(txbuf); }

	if (status_display) printf("* Disabling streaming channels\n");
//	if (rx0_i) { iio_channel_disable(rx0_i); }
//	if (rx0_q) { iio_channel_disable(rx0_q); }
	if (tx0_i) { iio_channel_disable(tx0_i); }
	if (tx0_q) { iio_channel_disable(tx0_q); }

	if (status_display) printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	if (status_display) printf("Waiting for process to finish... Got signal %d\n", sig);
	stop = true;
}

/* check return value of iio_attr_write function (for whole device) */
static void errchk_dev(int v) {
	if (v < 0) { fprintf(stderr, "Error %d writing to IIO device\n", v); shutdown(); }
}

/* check return value of iio_channel_attr_write function */
static void errchk_chn(int v, const char* what) {
	 if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk_chn(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk_chn(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(void)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
	IIO_ENSURE(dev && "No ad9361-phy found");
	return dev;
}

/* finds AD9361 streaming tx IIO devices */
static bool get_ad9361_tx_stream_dev(struct iio_device **dev)
{
	*dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
	return *dev != NULL;
}

/* finds AD9361 streaming tx IIO channels */
static bool get_ad9361_stream_ch(struct iio_device *dev, int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), 1);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), 1);
	return *chn != NULL;
}

/* finds AD9361 tx phy IIO configuration channel with id chid */
static bool get_tx_phy_chan( int chid, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("voltage", chid), true); 
	return *chn != NULL;
}

/* finds AD9361 local tx oscillator IIO configuration channels */
static bool get_tx_lo_chan(struct iio_channel **chn)
{
	*chn = iio_device_find_channel(get_ad9361_phy(), get_ch_name("altvoltage", 1), true);\
	return *chn != NULL;
}

/* finds AD9361 interpolation configuration channels */
static bool get_int8_chan(struct iio_channel **chn)
{
	struct iio_device *dev;

	IIO_ENSURE(get_ad9361_tx_stream_dev(&dev) && "No interpolation device found");

	*chn = iio_device_find_channel(dev, get_ch_name("voltage", 0), true);
	return *chn != NULL;
}

/* enables or disables Pluto's FPGA 8x interpolator for tx */
static bool set_pluto_8x_interpolator(bool enable)
{
	// The tx interpolator is set by channel-specific attributes
	// 		of the channel named voltage0
	// 		of the device named cf-ad9361-dds-core-lpc.
	// Two attributes are involved: sampling_frequency and sampling_frequency_available.
	// The attribute sampling_frequency_available returns a list of two available values,
	// which should differ by a factor of 8 (within a few counts).
	// The higher value is the sample clock rate when interpolation is not used;
	// the lower value is the effective sample clock rate when interpolation is used.
	// Set the attribute sampling_frequency to the value corresponding to the range of
	// sample rates you want to use.

	struct iio_channel *tx_int8_chn;
	long long sf1, sf2;

	if (status_display) printf("* %s 8x transmit interpolation\n", enable ? "Enabling" : "Disabling");

	// obtain the transmit interpolator's config channel,
	// namely device cf-ad9361-dds-core-lpc, channel voltage0
	if (!get_int8_chan(&tx_int8_chn)) { return false; }
	
	// read the attribute sampling_frequency_available,
	// which should be a list of two, like '30720000 3840000 '
	if(iio_channel_attr_read(	tx_int8_chn,
								"sampling_frequency_available",
								tmpstr, sizeof(tmpstr)
							) <= 0) {
		fprintf(stderr, "No sampling_frequency_available attribute\n");
		return false;
		}

	// parse the returned pair of numbers
	if (2 != sscanf(tmpstr, "%lld %lld ", &sf1, &sf2)) {
		fprintf(stderr, "sampling_frequency_available format unexpected\n");
		return false;
	}

	// validate that they're in approximate 8x ratio
	if (llabs(8 * sf2 - sf1) > 20) {
		fprintf(stderr, "sampling_frequency_available values not in ~8x ratio\n");
		return false;
	}

	// Now we can enable or disable interpolation
	if (enable) {
		// write the lower of the two values into attribute sampling_frequency
		// to set the interpolator to active.
		if (status_display) printf("* Writing %lld to set 8x interpolation\n", sf2);
		wr_ch_lli(tx_int8_chn, "sampling_frequency", sf2);
	} else {
		// write the higher of the two values into attribute sampling_frequency
		// to set the interpolator to inactive.
		if (status_display) printf("* Writing %lld to disable 8x interpolation\n", sf1);
		wr_ch_lli(tx_int8_chn, "sampling_frequency", sf1);
	}

	return true;
}

/* sets TX sample rate, always using Pluto's FPGA 8x interpolator */
static bool set_tx_sample_rate(struct iio_channel *phy_chn, long long sample_rate)
{
	long long ad9361_sample_rate;

	if (sample_rate < 260417) {
		fprintf(stderr, "Pluto can't sample slower than 260417 Hz\n");
		shutdown();
	} else if (sample_rate >= 2083333) {
		fprintf(stderr, "Pluto can't sample faster than 2.083333 MHz with the 8x interpolator enabled.\n");
		shutdown();
	}

	if (!set_pluto_8x_interpolator(1)) {
		fprintf(stderr, "Failed to set Pluto 8x interpolator\n");
		shutdown();
	}

	// Now we can set the sample rate at the AD9361, which is actually
	// the sample rate after interpolation.
	ad9361_sample_rate = sample_rate * 8;
	if (status_display) printf("* Setting AD9361 sample rate to %lld Hz\n", ad9361_sample_rate);
	wr_ch_lli(phy_chn, "sampling_frequency", ad9361_sample_rate);

	return true;
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_tx_ch(struct stream_cfg *cfg, int chid)
{
	struct iio_channel *chn = NULL;
	struct iio_channel *chn2 = NULL;

	// Configure phy and lo channels
	if (status_display) printf("* Acquiring AD9361 phy channel %d\n", chid);
	if (!get_tx_phy_chan(chid, &chn)) {
		return false;
		}
	wr_ch_str(chn, "rf_port_select", cfg->rfport);
	wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
	wr_ch_lli(chn, "hardwaregain", cfg->tx_gain);
	if (!set_tx_sample_rate(chn, cfg->fs_hz)) {
		return false;
		}

	// Configure LO channel
	if (status_display) printf("* Acquiring AD9361 tx lo channel\n");
	if (!get_tx_lo_chan(&chn2)) { return false; }
	wr_ch_lli(chn2, "frequency", cfg->lo_hz);
	return true;
}

/* turns off the transmit local oscillator
 *
 * This eliminates L.O. leakage through the antenna.
 */
static bool cfg_ad9361_txlo_powerdown(long long val)
{
	struct iio_channel *chn = NULL;
	
	if (!get_tx_lo_chan(&chn)) { return false; }
	wr_ch_lli(chn, "powerdown", val);
	return true;
}

/* adjusts single-ended crystal oscillator compensation for Pluto SDR */
static void cfg_ad9361_xo_correction(int delta)
{
	errchk_dev(iio_device_attr_write_longlong(get_ad9361_phy(), "xo_correction", 40000000 + delta));
}

/* turns off automatic transmit calibration for Pluto SDR */
static void cfg_ad9361_manual_tx_quad(void)
{
	errchk_dev(iio_device_attr_write(get_ad9361_phy(), "calib_mode", "manual_tx_quad"));
}


/* Obtain a sample from stdin */
int16_t get_next_sample(void)
{
	int16_t value;

	if (read(STDIN_FILENO, (void *)&value, 2) != 2) {	// detect EOF or error (dumbly)
		stop = true;
		return 0;
	}

	return value;
}


/* Convert a sample of FM deviation into an I/Q sample in an FM signal */
void modulate_sample(int16_t deviation, int16_t * const i_sample, int16_t * const q_sample)
{
    static double signal = 0.0;	//start the signal at 0, keep its instantaneous value here.

	double deviation_in_hertz = deviation * deviation_scale_factor + offset_lo_offset;
	double phase_increment_this_sample = 2 * M_PI * deviation_in_hertz * time_per_sample;

    signal = fmod(signal + phase_increment_this_sample, 2 * M_PI);
    *i_sample = (int16_t)(cos(signal) * MAX_SAMPLE_VALUE);  // scale to 16-bit integer (12 MSbits used)
    *q_sample = (int16_t)(sin(signal) * MAX_SAMPLE_VALUE);
}


void usage(void)
{
	fprintf(stderr,
		"pluto_tx_fm, FM transmitter for ADALM-Pluto\n\n"
		"Usage:\tpluto_tx_fm -f freq  -s samplerate [-options]\n\n"
		"\t-f center_frequency\n"
		"\t\tCenter frequency in Hz (no default)\n\n"

		"\t-s samplerate\n"
		"\t\tSample rate in Hz. Used on both the input stream and the transmitted output.\n\n"

		"\t-u iio_context_url\n"
		"\t\tURL of the Pluto device, in libiio format.\n"
		"\t\tDefault: ip:pluto.local\n\n"

		"\t-d deviation\n"
		"\t\tDesired FM deviation corresponding to the maximum positive sample value 0x7FFF.\n"
		"\t\tThis is internally translated to a sensitivity factor.\n"
		"\t\tSmaller sample values result in proportionally smaller deviations;\n"
		"\t\tnegative sample values result in negative deviations.\n"
		"\t\tDefault deviation 10000.\n\n"

		"\t-a transmit_attenuation\n"
		"\t\tspecifies in dB the amount by which the output power of the Pluto\n"
		"\t\tshould be reduced from full scale. Default is 10, the minimum attentuation\n"
		"\t\tthat is safe to loop back into the Pluto's receiver.\n\n"

		"\t-b buffer_size\n"
		"\t\tspecifies the size in samples of the IIO kernel buffers.\n"
		"\t\tDefault size is 0.040 * samplerate, for 40ms buffering.\n\n"

		"\t-x xo_correction\n"
		"\t\tspecifies the crystal oscillator frequency correction in Hz.\n"
		"\t\tDefault is 0.\n\n"

		"\t-E\n"
		"\t\tEnable offset tuning, moving the Pluto's local oscillator frequency -1.5*deviation\n\n"
		
		"\t-q\n"
		"\t\tQuiet status output\n\n"
		);
	exit(1);
}



int main (int argc, char **argv)
{
	// Streaming devices
	struct iio_device *tx;

	// TX sample counter
	size_t ntx = 0;
	
	// Buffer pointers
	char *p_dat, *p_end;
	ptrdiff_t p_inc;

	// Stream configuration
	struct stream_cfg txcfg;

	// Listen to ctrl+c and IIO_ENSURE
	signal(SIGINT, handle_sig);

	int opt;
	while ((opt = getopt(argc, argv, "f:s:u:d:a:b:x:hqE")) != -1) {
		switch (opt) {
			case 'f':
				center_frequency = (long long)atof(optarg);
				break;
			
			case 's':
				sample_rate = (long long)atof(optarg);
				break;
			
			case 'u':
				strncpy(iio_context_url, optarg, MAX_CONTEXT_URL_LEN);
				break;
			
			case 'd':
				max_deviation = (long) atof(optarg);
				break;
			
			case 'a':
				transmit_attenuation = (int)atof(optarg);
				break;
			
			case 'b':
				buffer_size = (unsigned long)atof(optarg);
				break;

			case 'x':
				xo_correction = (int)atof(optarg);
				break;

			case 'q':
				status_display = 0;
				break;

			case 'E':
				offset_lo = 1;
				break;
			
			case 'h':
			default:
				usage();
				break;
		}
	}

	if (center_frequency == -1) {
		fprintf(stderr, "You must supply a center frequency (-f freq).\n");
		exit(1);
	} else if (center_frequency < 70e6) {
		fprintf(stderr, "Frequency %lld is lower than the Pluto's limit of 70 MHz.\n", center_frequency);
		exit(1);
	} else if (center_frequency >= 6e9) {
		fprintf(stderr, "Frequency %lld is higher than the Pluto's limit of 6 GHz.\n", center_frequency);
		exit(1);
	} else {
		txcfg.lo_hz = center_frequency;	// RF center frequency
		if (status_display) printf("* Center frequency = %lld\n", center_frequency);
	}

	if (sample_rate == -1) {
		fprintf(stderr, "You must supply a sample rate (-s samplerate).\n");
		exit(1);
	} else if (sample_rate < 260417) {
		fprintf(stderr, "Sample rate %lld is lower than the Pluto's limit of 260418 Hz.\n", sample_rate);
		exit(1);
	} else if (sample_rate >= 2083333) {
		fprintf(stderr, "Sample rate %lld is higher than the Pluto's limit (with interpolation) of 2083333 Hz.\n", center_frequency);
		exit(1);
	} else {
		txcfg.fs_hz = sample_rate;	// baseband sample rate
		time_per_sample = 1.0 / sample_rate;
		if (status_display) printf("* Sample rate = %lld\n", sample_rate);
	}

	ctx = iio_create_context_from_uri(iio_context_url);
	if (!ctx) {
		fprintf(stderr, "Could not create context %s\n", iio_context_url);
		exit(1);
	}

	if (max_deviation < 100 || max_deviation > 100000) {
		fprintf(stderr, "deviation %ld is unreasonable.\n", max_deviation);
		exit(1);
	} else {
		deviation_scale_factor = (double)max_deviation / MAX_SAMPLE_VALUE;
	}

	if (transmit_attenuation < 0 || transmit_attenuation > 89) {
		fprintf(stderr, "Transmit attenuation %d out of range (0-89)\n\n", transmit_attenuation);
		exit(1);
	} else {
		txcfg.tx_gain = -transmit_attenuation;
		if (status_display) printf("* Transmit attenuation = %d dB\n", transmit_attenuation);
	}

	if (buffer_size == 0) {
		buffer_size = (size_t)(0.040 * sample_rate);	// 40ms buffer if not specified
	}
	if (status_display) printf("* Buffer size = %lu bytes\n", buffer_size);

	if (offset_lo) {
		offset_lo_offset = 1.5 * max_deviation;
		txcfg.lo_hz = center_frequency - offset_lo_offset;	// LO offset from RF center frequency
		if (status_display) printf("* Offset LO frequency = %lld\n", txcfg.lo_hz);
	}

	// TX stream config constant values
	txcfg.bw_hz = 200000;	// 200 kHz RF bandwidth, Pluto's minimum
	txcfg.rfport = "A"; // port A (select for rf freq.)

	if (status_display) printf("* Created IIO context %s\n", iio_context_url);
	IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");
	unsigned int attrs_count = iio_context_get_attrs_count(ctx);
	IIO_ENSURE(attrs_count > 0 && "No context attributes");

	if (status_display) printf("* Acquiring AD9361 streaming devices\n");
	IIO_ENSURE(get_ad9361_tx_stream_dev(&tx) && "No tx dev found");

	if (status_display) printf("* Configuring Pluto SDR for transmitting\n");
	cfg_ad9361_manual_tx_quad();	// disable automatic TX calibration
	cfg_ad9361_xo_correction(xo_correction);	// -465 out of 40e6 for remote lab's Pluto S/N b83991001015001f00c7a0653f04
	cfg_ad9361_txlo_powerdown(0);	// enable transmit LO

	if (status_display) printf("* Configuring AD9361 for streaming\n");
	IIO_ENSURE(cfg_ad9361_streaming_tx_ch(&txcfg, 0) && "TX port 0 not found");

	if (status_display) printf("* Initializing AD9361 IIO streaming channels\n");
	IIO_ENSURE(get_ad9361_stream_ch(tx, 0, &tx0_i) && "TX chan i not found");
	IIO_ENSURE(get_ad9361_stream_ch(tx, 1, &tx0_q) && "TX chan q not found");

	if (status_display) printf("* Enabling IIO streaming channels\n");
	iio_channel_enable(tx0_i);
	iio_channel_enable(tx0_q);
	
	if (status_display) printf("* Creating a non-cyclic IIO buffer of %lu samples\n", buffer_size);

	txbuf = iio_device_create_buffer(tx, buffer_size, false);
	if (!txbuf) {
		perror("Could not create TX buffer");
		shutdown();
		return 0;
	}
	
	if (status_display) printf("* Ready to transmit\n");





	/*	!!! TODO: redo the streaming logic for real-world use case, including
		intermittent transmitting with PTT, imperfect sample rates, etc.

		The goal is to accept one or more bursts of realtime data at approximately
		the nominal sample rate, while doing something reasonable when these
		expectations are not met.

		We cannot assume that our source and sink are in lock step. If both have
		realtime pacing, they may have any alignment and that alignment may drift.
		The source may not be paced at all, and it might be too slow or more than
		fast enough. If the source is too slow, there isn't much we can do except
		to diagnose the underrun condition, but we don't want to do this accidentally
		in case of a predictable transient event. If the source is too fast, but
		smart about buffering (as, presumably, a shell pipeline would be) there's
		no problem.

		We have a couple of common use cases.
			1. Transmit from a file using a shell pipeline. In this case, the source
			is presumably fast, and we always have plenty of input until EOF. When the
			file ends, we can just exit.

			2. Transmit from a realtime PTT source. In this case, the source is
			intermittent. We need to be transmitting only when we have source data,
			so we start out silent, enable the transmitter when we're ready, and
			cleanly finish the transmission and shut off the transmitter when the
			data runs dry. After which, we go back to silent mode until a new
			transmission arrives. If there's underrun during a transmission, that's
			bad. We don't have access to higher protocol layers to fix this. 


		At startup, we are waiting for a buffer of samples to arrive. state = WAIT.
		In WAIT, we are blocked on stdin.

		If a sample arrives on stdin while in WAIT, we add it to the empty input buffer,
		transition to the PARTIAL state, and set (or reset) the partial input timeout.

		If EOF occurs on stdin in the WAIT state, we should just exit.
		This is the normal termination for a file transmission, for instance. I think.

		If some other error occurs on stdin in the WAIT state, we have a broken pipeline
		and should try to print a diagnostic error message to stderr before exiting.

		If ^C happens in the WAIT state, we should 

		The only other things that can happen in WAIT are ^C or EOF on stdin. 

		If the timeout expires, we fill the rest of the buffer with 0 and push it,
		transitioning to FINISH state.

		If a sample arrives in ...
	

		For now, we have just hacked up the old logic:	
	*/



	// Write first TX buf with all zeroes, for cleaner startup 
	p_inc = iio_buffer_step(txbuf);
	p_end = iio_buffer_end(txbuf);
	for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			((int16_t*)p_dat)[0] = 0 << 4; // Real (I)
			((int16_t*)p_dat)[1] = 0 << 4; // Imag (Q)
		}

	cfg_ad9361_txlo_powerdown(0);

	if (status_display) printf("* Starting tx streaming (press CTRL+C to cancel)\n");
	while (!stop)
	{
		ssize_t nbytes_tx;
		int16_t sample;

		// Schedule TX buffer
		nbytes_tx = iio_buffer_push(txbuf);
		if (nbytes_tx < 0) { fprintf(stderr, "Error pushing buf %d\n", (int) nbytes_tx); shutdown(); }

		// WRITE: Get pointers to TX buf and write IQ to TX buf port 0
		p_inc = iio_buffer_step(txbuf);
		p_end = iio_buffer_end(txbuf);
		for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			sample = get_next_sample();
			modulate_sample(sample, (int16_t*)p_dat, (int16_t*)(p_dat+2));
		}

		// Sample counter increment and status output
		ntx += nbytes_tx / iio_device_get_sample_size(tx);
		if (status_display) {
			printf("\tTX %8.2f MSmp\r", ntx/1e6);
			fflush(stdout);
		}
	}
	if (status_display) printf("\n");

	cfg_ad9361_txlo_powerdown(1);
	shutdown();

	return 0;
}
