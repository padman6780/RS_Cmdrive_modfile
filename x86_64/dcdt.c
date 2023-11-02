/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__dcdt
#define _nrn_initial _nrn_initial__dcdt
#define nrn_cur _nrn_cur__dcdt
#define _nrn_current _nrn_current__dcdt
#define nrn_jacob _nrn_jacob__dcdt
#define nrn_state _nrn_state__dcdt
#define _net_receive _net_receive__dcdt 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define c1 _p[0]
#define c2 _p[1]
#define w _p[2]
#define tbegin _p[3]
#define tdur _p[4]
#define dc _p[5]
#define i _p[6]
#define v _p[7]
#define _g _p[8]
#define c	*_ppvar[0]._pval
#define _p_c	_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  0;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_cm(void);
 static void _hoc_dcmdt(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_dcdt", _hoc_setdata,
 "cm_dcdt", _hoc_cm,
 "dcmdt_dcdt", _hoc_dcmdt,
 0, 0
};
#define cm cm_dcdt
#define dcmdt dcmdt_dcdt
 extern double cm( _threadargsprotocomma_ double );
 extern double dcmdt( _threadargsprotocomma_ double );
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "c1_dcdt", "uF/cm2",
 "c2_dcdt", "uF/cm2",
 "w_dcdt", "/ms",
 "tbegin_dcdt", "ms",
 "tdur_dcdt", "ms",
 "dc_dcdt", "uF/cm2-ms",
 "i_dcdt", "mA/cm2",
 "c_dcdt", "uF/cm2",
 0,0
};
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void _ba1(Node*_nd, double* _pp, Datum* _ppd, Datum* _thread, NrnThread* _nt) ;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"dcdt",
 "c1_dcdt",
 "c2_dcdt",
 "w_dcdt",
 "tbegin_dcdt",
 "tdur_dcdt",
 "dc_dcdt",
 0,
 "i_dcdt",
 0,
 0,
 "c_dcdt",
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 9, _prop);
 	/*initialize range parameters*/
 	c1 = 1;
 	c2 = 0;
 	w = 0;
 	tbegin = 1e+09;
 	tdur = 0;
 	dc = 0;
 	_prop->param = _p;
 	_prop->param_size = 9;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _dcdt_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 9, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "pointer");
 	hoc_reg_ba(_mechtype, _ba1, 11);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 dcdt /home/mithun/Documents/Work/DUK/CAN_Lab/positions/DST_CSRI_PDF/project/03_standalone_BLS_NEURON_model_python/03f_Cmdrive_NEURON/src/Collab_wth_Hines/My_attempts/RS_Cmdrive_mod_working/dcdt.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define PI _nrnunit_PI[_nrnunit_use_legacy_]
static double _nrnunit_PI[2] = {0x1.921fb54442d18p+1, 3.14159}; /* 3.14159265358979312 */
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
double cm ( _threadargsprotocomma_ double _lt ) {
   double _lcm;
 if ( _lt >= tbegin  && _lt <= ( tbegin + tdur ) ) {
     _lcm = c1 + c2 * sin ( 2.0 * PI * w * ( _lt - tbegin ) ) ;
     }
   else {
     _lcm = c1 ;
     }
   
return _lcm;
 }
 
static void _hoc_cm(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  cm ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double dcmdt ( _threadargsprotocomma_ double _lt ) {
   double _ldcmdt;
 if ( _lt >= tbegin  && _lt <= ( tbegin + tdur ) ) {
     _ldcmdt = c2 * ( sin ( 2.0 * PI * w * ( _lt - tbegin + dt ) ) - sin ( 2.0 * PI * w * ( _lt - tbegin ) ) ) / dt ;
     }
   else {
     _ldcmdt = 0.0 ;
     }
   
return _ldcmdt;
 }
 
static void _hoc_dcmdt(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  dcmdt ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 /* BEFORE BREAKPOINT */
 static void _ba1(Node*_nd, double* _pp, Datum* _ppd, Datum* _thread, NrnThread* _nt)  {
   double* _p; Datum* _ppvar; _p = _pp; _ppvar = _ppd;
  v = NODEV(_nd);
 c = cm ( _threadargscomma_ t ) ;
   dc = dcmdt ( _threadargscomma_ t ) ;
   }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
 {
   dc = 0.0 ;
   }

}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   at_time ( _nt, tbegin ) ;
   at_time ( _nt, tbegin + tdur ) ;
   i = dc * v * ( 0.001 ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/home/mithun/Documents/Work/DUK/CAN_Lab/positions/DST_CSRI_PDF/project/03_standalone_BLS_NEURON_model_python/03f_Cmdrive_NEURON/src/Collab_wth_Hines/My_attempts/RS_Cmdrive_mod_working/dcdt.mod";
static const char* nmodl_file_text = 
  ": Provides an example of how to manage variable capacitance using\n"
  ": the relation q = c*v so that i = (c*dv/dt) + (dc/dt * v)\n"
  ": The capacitance is assumed to be c(t)\n"
  ": The effect of changing capacitance on the (c*dv/dt) term is accomplished\n"
  ": via a POINTER to the compartment cm (set up in hoc) where the c pointer\n"
  ": is assigned a value in the BEFORE BREAKPOINT block.  The effect of\n"
  ": the (dc/dt * v) term is accomplished in the BREAKPOINT block.\n"
  "\n"
  "UNITS {\n"
  "  (mA) = (milliamp)\n"
  "  (mV) = (millivolt)\n"
  "  (uF) = (microfarad)\n"
  "  PI = (pi) (1)\n"
  "}\n"
  "\n"
  "NEURON {\n"
  "  SUFFIX dcdt\n"
  "  THREADSAFE\n"
  "  RANGE c1, c2, w, tbegin, tdur, dc\n"
  "  POINTER c\n"
  "  NONSPECIFIC_CURRENT i\n"
  "}\n"
  " \n"
  "PARAMETER {\n"
  "  c1 = 1 (uF/cm2)\n"
  "  c2 = 0 (uF/cm2)\n"
  "  w = 0 (/ms)\n"
  "  tbegin = 1e9 (ms) \n"
  "  tdur = 0 (ms)\n"
  "  dc (uF/cm2-ms)\n"
  "}\n"
  " \n"
  "ASSIGNED {\n"
  "  c (uF/cm2)\n"
  "  i (mA/cm2)\n"
  "  v (mV)\n"
  "  dt (ms)\n"
  "}\n"
  " \n"
  "FUNCTION cm(t(ms)) (uF/cm2) {\n"
  "  if (t >= tbegin && t <= (tbegin + tdur)) {\n"
  "    cm = c1 + c2*sin(2*PI*w*(t - tbegin))\n"
  "  }else{\n"
  "    cm = c1\n"
  "  }\n"
  "}\n"
  "        \n"
  "FUNCTION dcmdt(t(ms))(uF/cm2-ms) {\n"
  "  if (t >= tbegin && t <= (tbegin + tdur)) {\n"
  "    \n"
  "    dcmdt = c2*(sin(2*PI*w*(t-tbegin + dt)) - sin(2*PI*w* ( t - tbegin)) ) / dt \n"
  "  }else{\n"
  "    dcmdt = 0\n"
  "  }\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "  dc = 0\n"
  "}\n"
  "\n"
  "BEFORE BREAKPOINT {\n"
  "  c = cm(t)\n"
  "  dc = dcmdt(t)\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "  at_time(tbegin)\n"
  "  at_time(tbegin + tdur)\n"
  "  \n"
  "  i = dc*v*(0.001)\n"
  "}\n"
  ;
#endif
