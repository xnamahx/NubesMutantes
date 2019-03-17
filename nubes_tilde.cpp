#include "c74_msp.h"
#include "clouds/dsp/granular_processor.h"
#include <iostream>

using namespace c74::max;

static const char* clds_version = "0.5"; 

static t_class* this_class = nullptr;

inline double constrain(double v, double vMin, double vMax) {
	return std::max<double>(vMin, std::min<double>(vMax, v));
}

inline int min(int a, int b) {
	return (a < b) ? a : b;
}

/** Returns the maximum of `a` and `b` */
inline int max(int a, int b) {
	return (a > b) ? a : b;
}

inline int clamp(int x, int a, int b) {
	return min(max(x, a), b);
}

inline short TO_SHORTFRAME(float v) { return (short(v * 16384.0f)); }
inline float FROM_SHORTFRAME(short v) { return (float(v) / 16384.0f); }

struct t_nubes {
	t_object  x_obj;

	double  f_dummy;

	double f_freeze;
	double f_trig;
	double f_position;
	double f_size;
	double f_pitch;
	double f_density;
	double f_texture;
	double f_mix;
	double f_spread;
	double f_feedback;
	double f_reverb;
	double f_mode;
	double f_mono;
	double f_silence;
	double f_bypass;
	double f_lofi;
	double f_num_channels;
	float 	f_sample_rate;

	//inlet<>  x_in_left{ this, "(signal) Sample index" };
	/*t_inlet*  x_in_right{ this, "(signal) Sample index" };
	t_outlet* x_out_left{ this, "(signal) Sample value at index", "signal" };
	t_outlet* x_out_right{ this, "(signal) Sample value at index", "signal" };*/
	// CLASS_MAINSIGNALIN  = in_left;
	/*t_inlet*  x_in_right;
	t_outlet* x_out_left;
	t_outlet* x_out_right;*/

	clouds::GranularProcessor processor;
	bool ltrig;
	clouds::ShortFrame* ibuf;
	clouds::ShortFrame* obuf;
	int iobufsz;

/*	static const int LARGE_BUF = 524288;
	static const int SMALL_BUF = 262144;*/
	static const int LARGE_BUF = 118784;
	static const int SMALL_BUF = 65536 - 128;
	uint8_t* large_buf;
	int      large_buf_size;
	uint8_t* small_buf;
	int      small_buf_size;
};



void nubes_perform64(t_nubes* self, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam) {
    if (numouts>0 && numins>0)
    {
	    double    *in = ins[0];     // first inlet
	    double    *in2 = ins[1];     // first inlet
	    double    *out = outs[0];   // first outlet
	    double    *out2 = outs[1];   // first outlet

		/*if (sampleframes > self->iobufsz) {
			delete[] self->ibuf;
			delete[] self->obuf;
			self->iobufsz = sampleframes;
			self->ibuf = new clouds::ShortFrame[self->iobufsz];
			self->obuf = new clouds::ShortFrame[self->iobufsz];
		}*/

		for (auto i=0; i<self->iobufsz; ++i){
			self->ibuf[i].l = clamp((*in++) * 32767.0f, -32768.0f, 32767.0f);
			self->ibuf[i].r = clamp((*in2++) * 32767.0f, -32768.0f, 32767.0f);
		}

		self->processor.Prepare();
		self->processor.Process(self->ibuf, self->obuf, self->iobufsz);

		for (int i = 0; i < self->iobufsz; i++) {
			*out++ = self->obuf[i].l / 32768.0;
	        *out2++ = self->obuf[i].r / 32768.0;
		}

	}
}

void* nubes_new(void) {
	t_nubes* self = (t_nubes*)object_alloc(this_class);


	self->iobufsz = 64;
	self->ibuf = new clouds::ShortFrame[self->iobufsz];
	self->obuf = new clouds::ShortFrame[self->iobufsz];
	self->large_buf_size = t_nubes::LARGE_BUF;
	self->large_buf = new uint8_t[self->large_buf_size];
	self->small_buf_size = t_nubes::SMALL_BUF;
	self->small_buf = new uint8_t[self->small_buf_size];
/*
	self->f_freeze = 0.0f;
	self->f_trig = 0.0f;
	self->f_position = 0.0f;
	self->f_size = 0.0f;
	self->f_pitch = 0.0f;
	self->f_density = 0.0f;
	self->f_texture = 0.0f;
	self->f_mix = 0.0f;
	self->f_spread = 0.0f;
	self->f_feedback = 0.0f;
	self->f_reverb = 0.0f;
	self->f_mode = 0.0f;
	self->f_mono = 0.0f;
	self->f_silence = 0.0f;
	self->f_bypass = 0.0f;
	self->f_lofi = 0.0f;
*/
	self->processor.Init(self->large_buf,self->LARGE_BUF,self->small_buf,self->SMALL_BUF);

	outlet_new(self, "signal");
	outlet_new(self, "signal");
	inlet_new(self, NULL);

	dsp_setup((t_pxobject*)self, 2);

	return (void *)self;
}


void nubes_free(t_nubes* self) {
	dsp_free((t_pxobject*)self);
}

void nubes_dsp64(t_nubes* self, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags) {
	object_method_direct(void, (t_object*, t_object*, t_perfroutine64, long, void*),
						 dsp64, gensym("dsp_add64"), (t_object*)self, (t_perfroutine64)nubes_perform64, 0, NULL);
}


void nubes_assist(t_nubes* self, void* unused, t_assist_function io, long index, char* string_dest) {
	if (io == ASSIST_INLET) {
		switch (index) {
			case 1: 
				strncpy(string_dest,"(signal) L IN", ASSIST_STRING_MAXSIZE); 
				break;
			case 2: 
				strncpy(string_dest,"(signal) R IN", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
	else if (io == ASSIST_OUTLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"(signal) L Output", ASSIST_STRING_MAXSIZE); 
				break;
			case 1: 
				strncpy(string_dest,"(signal) R Output", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
}


void nubes_freeze(t_nubes *x, double f)
{
  	x->f_freeze = f;
	x->processor.mutable_parameters()->freeze = (x->f_freeze > 0.5f);
}
void nubes_trig(t_nubes *x, double f)
{
  	x->f_trig = f;
	//note the trig input is really a gate... which then feeds the trig
	x->processor.mutable_parameters()->gate = (x->f_trig > 0.5f);
	x->processor.mutable_parameters()->trigger = (x->f_trig > 0.5f);
}
void nubes_position(t_nubes *x, double f)
{
	x->f_position = f;
	x->processor.mutable_parameters()->position = constrain(x->f_position, 0.0f, 1.0f);
}
void nubes_size(t_nubes *x, double f)
{
	x->f_size = f;
	x->processor.mutable_parameters()->size = constrain(x->f_size, 0.0f, 1.0f);
}
void nubes_pitch(t_nubes *x, double f)
{
  	x->f_pitch = f;
	x->processor.mutable_parameters()->pitch = constrain(x->f_pitch * 64.0f, -64.0f, 64.0f);
}
void nubes_density(t_nubes *x, double f)
{
  	x->f_density = f;
	double density = constrain(x->f_density, 0.0f, 1.0f);
	density = (x->f_mode == clouds::PLAYBACK_MODE_GRANULAR) ? (density*0.6f) + 0.2f : density;
	x->processor.mutable_parameters()->density = constrain(density, 0.0f, 1.0f);
}

void nubes_texture(t_nubes *x, double f)
{
	x->f_texture = f;
	x->processor.mutable_parameters()->texture = constrain(x->f_texture, 0.0f, 1.0f);
}

void nubes_mix(t_nubes *x, double f)
{
	x->f_mix = f;
	x->processor.mutable_parameters()->dry_wet = constrain(x->f_mix, 0.0f, 1.0f);
}

void nubes_spread(t_nubes *x, double f)
{
  	x->f_spread = f;
	x->processor.mutable_parameters()->stereo_spread = constrain(x->f_spread, 0.0f, 1.0f);
}

void nubes_feedback(t_nubes *x, double f)
{
  	x->f_feedback = f;
	x->processor.mutable_parameters()->feedback = constrain(x->f_feedback, 0.0f, 1.0f);
}

void nubes_reverb(t_nubes *x, double f)
{
  	x->f_reverb = f;
	x->processor.mutable_parameters()->reverb = constrain(x->f_reverb, 0.0f, 1.0f);
}

void nubes_mode(t_nubes *x, double f)
{
  	x->f_mode = f;
	clouds::PlaybackMode mode = (clouds::PlaybackMode) (int(x->f_mode) % clouds::PLAYBACK_MODE_LAST);
	if (mode != x->processor.playback_mode()) {
		x->processor.set_playback_mode(mode);
		switch (mode) {
		case clouds::PLAYBACK_MODE_GRANULAR: object_post(&x->x_obj,"clds:granular"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_STRETCH: object_post(&x->x_obj,"clds:stretch"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_LOOPING_DELAY: object_post(&x->x_obj,"clds:looping"); x->processor.set_num_channels(2); break;
		case clouds::PLAYBACK_MODE_SPECTRAL: object_post(&x->x_obj,"clds:spectral"); x->processor.set_num_channels(1); break;
		case clouds::PLAYBACK_MODE_LAST:
		default: object_post(&x->x_obj,"clds : unknown mode");
		}
	}
}

void nubes_mono(t_nubes *x, double f)
{
  x->f_mono = f;
}

void nubes_silence(t_nubes *x, double f)
{
  	x->f_silence = f;
	x->processor.set_silence(x->f_silence > 0.5f);
}

void nubes_bypass(t_nubes *x, double f)
{
  	x->f_bypass = f;
	x->processor.set_bypass(x->f_bypass > 0.5f);
}

void nubes_lofi(t_nubes *x, double f)
{
  	x->f_lofi = f;
	x->processor.set_low_fidelity(x->f_lofi > 0.5f);
}


void nubes_numchannels(t_nubes *x, double f)
{
  	x->f_num_channels = f;
	x->processor.set_num_channels(x->f_num_channels);
}

void nubes_samplerate(t_nubes *x, double f)
{
  	x->f_sample_rate = f;
	x->processor.sample_rate(x->f_sample_rate);
}

void ext_main(void* r) {
	this_class = class_new("nubes~", (method)nubes_new, (method)nubes_free, sizeof(t_nubes), NULL, A_GIMME, 0);

	class_addmethod(this_class,(method) nubes_assist, "assist",	A_CANT,		0);
	class_addmethod(this_class,(method) nubes_dsp64, "dsp64",	A_CANT,		0);
	
	class_addmethod(this_class,(method) nubes_freeze, "freeze", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_trig, "trig", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_position, "position", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_size, "size", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_pitch, "pitch", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_density, "density", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_texture, "texture", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_mix, "mix", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_spread, "spread", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_feedback, "feedback", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_reverb, "reverb", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_mode, "mode", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_mono, "mono", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_silence, "silence", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_bypass, "bypass", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_lofi, "lofi", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_samplerate, "samplerate", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) nubes_numchannels, "numchannels", A_DEFFLOAT, 0);

	class_dspinit(this_class);
	class_register(CLASS_BOX, this_class);
}