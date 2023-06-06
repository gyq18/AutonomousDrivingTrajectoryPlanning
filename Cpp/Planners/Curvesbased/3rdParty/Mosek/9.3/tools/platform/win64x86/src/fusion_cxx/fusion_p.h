#ifndef __FUSION_P_H__
#define __FUSION_P_H__
#include "monty.h"
#include "mosektask_p.h"
#include "list"
#include "vector"
#include "unordered_map"
#include "fusion.h"
namespace mosek
{
namespace fusion
{
// mosek.fusion.BaseModel from file 'src\fusion\cxx\BaseModel_p.h'
// namespace mosek::fusion
struct p_BaseModel
{
  p_BaseModel(BaseModel * _pubthis);

  void _initialize( monty::rc_ptr<BaseModel> m);
  void _initialize( const std::string & name,
                    const std::string & licpath);

  virtual ~p_BaseModel() { /* std::cout << "~p_BaseModel()" << std::endl;*/  }

  static p_BaseModel * _get_impl(Model * _inst) { return _inst->_impl; }

  //----------------------

  bool synched;
  std::string taskname;

  monty::rc_ptr<SolutionStruct> sol_itr;
  monty::rc_ptr<SolutionStruct> sol_itg;
  monty::rc_ptr<SolutionStruct> sol_bas;

  //---------------------

  std::unique_ptr<Task> task;

  //---------------------
  void task_setLogHandler (const msghandler_t & handler);
  void task_setDataCallbackHandler (const datacbhandler_t & handler);
  void task_setCallbackHandler (const cbhandler_t & handler);

  int task_append_barvar(int size, int num);

  void task_var_name   (int index, const std::string & name);
  void task_con_name   (int index, const std::string & name);
  void task_cone_name  (int index, const std::string & name);
  void task_barvar_name(int index, const std::string & name);
  void task_objectivename(         const std::string & name);

  void task_format_var_names(const std::shared_ptr<monty::ndarray<int,1>> subj, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);
  void task_format_con_names(const std::shared_ptr<monty::ndarray<int,1>> subi, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);
  void task_format_cone_names(const std::shared_ptr<monty::ndarray<int,1>> subk, const std::string & format,const std::shared_ptr<monty::ndarray<int,1>> dims, const std::shared_ptr<monty::ndarray<long long,1>> sp);

  void task_break_solve();

  //--------------------------

  int task_numvar();
  int task_numcon();
  int task_numcone();
  int task_numbarvar();

  //--------------------------

  void task_put_param(const std::string & name, const std::string & value);
  void task_put_param(const std::string & name, int    value);
  void task_put_param(const std::string & name, double value);
  
  double    task_get_dinf (const std::string & name);
  int       task_get_iinf (const std::string & name);
  long long task_get_liinf(const std::string & name);
  
  //--------------------------

  void task_con_putboundlist_fr(const std::shared_ptr<monty::ndarray<int,1>> idxs);
  void task_con_putboundlist_lo(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_up(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_fx(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_con_putboundlist_ra(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & lb , 
                                const std::shared_ptr<monty::ndarray<double,1>> & ub );

  void task_var_putboundlist_fr(const std::shared_ptr<monty::ndarray<int,1>> idxs);
  void task_var_putboundlist_lo(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_up(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_fx(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & rhs);
  void task_var_putboundlist_ra(const std::shared_ptr<monty::ndarray<int,1>> idxs, const std::shared_ptr<monty::ndarray<double,1>> & lb , 
                                const std::shared_ptr<monty::ndarray<double,1>> & ub );
  
  void task_var_putintlist(const std::shared_ptr<monty::ndarray<int,1>> & idxs);
  void task_var_putcontlist(const std::shared_ptr<monty::ndarray<int,1>> & idxs); 
 
  //--------------------------

  void task_putbararowlist(const std::shared_ptr<monty::ndarray<int,1>>       subi,
                           const std::shared_ptr<monty::ndarray<long long,1>> ptr,
                           const std::shared_ptr<monty::ndarray<int,1>>       subj,
                           const std::shared_ptr<monty::ndarray<long long,1>> matidx);

  void task_putbaraijlist(const std::shared_ptr<monty::ndarray<int,1>> subi,
                          const std::shared_ptr<monty::ndarray<int,1>> subj,
                          std::shared_ptr<monty::ndarray<long long,1>> matidx);
  
  void task_putbarc(const std::shared_ptr<monty::ndarray<int,1>> subj,
                    const std::shared_ptr<monty::ndarray<int,1>> subl,
                    const std::shared_ptr<monty::ndarray<int,1>> subk,
                    const std::shared_ptr<monty::ndarray<double,1>> val);
  
  std::shared_ptr<monty::ndarray<long long,1>> task_appendsymmatlist (const std::shared_ptr<monty::ndarray<int,1>>       & dim, 
                                                                      const std::shared_ptr<monty::ndarray<long long,1>> & nz, 
                                                                      const std::shared_ptr<monty::ndarray<int,1>>       & subk, 
                                                                      const std::shared_ptr<monty::ndarray<int,1>>       & subl, 
                                                                      const std::shared_ptr<monty::ndarray<double,1>>    & val);
  int  task_barvar_dim(int j);
  void task_putbaraij (int i, int j, int k);
  void task_putbaraij (int i, int j, const std::shared_ptr<monty::ndarray<int,1>> & k);
  void task_putbarcj  (int j, int k);
  void task_putbarcj  (int j,        const std::shared_ptr<monty::ndarray<int,1>> & k);
  int  task_barvardim (int index);

  int task_append_var(int num);
  int task_append_con(int num);

  void task_append_zerocones (int numcone);
  void task_clear_cones   ( const std::shared_ptr<monty::ndarray<int,1>> & idxs );
  void task_put_zerocones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_quadcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_rquadcones( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_pexpcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_ppowcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs, const std::shared_ptr<monty::ndarray<double,1>> & alpha );
  void task_put_dexpcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs );
  void task_put_dpowcones ( const std::shared_ptr<monty::ndarray<int,1>> & idxs, int conesize, int numcone,  const std::shared_ptr<monty::ndarray<int,1>> & membs, const std::shared_ptr<monty::ndarray<double,1>> & alpha );

  void task_cleararowlist(const std::shared_ptr<monty::ndarray<int,1>> & idxs);
  void task_clearacollist(const std::shared_ptr<monty::ndarray<int,1>> & idxs);

  void task_putarowlist(
    const std::shared_ptr<monty::ndarray<int,1>>       & idxs, 
    const std::shared_ptr<monty::ndarray<long long,1>> & ptrb, 
    const std::shared_ptr<monty::ndarray<int,1>>       & subj, 
    const std::shared_ptr<monty::ndarray<double,1>>    & cof);
  void task_putaijlist(
    const std::shared_ptr<monty::ndarray<int,1>>       & subi, 
    const std::shared_ptr<monty::ndarray<int,1>>       & subj, 
    const std::shared_ptr<monty::ndarray<double,1>>    & cof, 
    long long                           num);

  void task_setnumvar(int num);
  void task_cleanup(int oldnum, int oldnumcon, int oldnumcone, int oldnumbarvar);
  void task_putoptserver_host(const std::string & addr);
  void task_solve(bool remote, const std::string & server, const std::string & port);

  void task_putobjective( 
    bool                             maximize,
    const std::shared_ptr<monty::ndarray<int,1>>    & subj    ,
    const std::shared_ptr<monty::ndarray<double,1>> & cof     ,
    double                           cfix    );

  void task_putclist(   
    const std::shared_ptr<monty::ndarray<int,1>>    & subj,
    const std::shared_ptr<monty::ndarray<double,1>> & cof);


  void task_putobjectivename(const std::string & name);

  void task_write(const std::string & filename);
  void task_dump (const std::string & filename);

  MSKtask_t task_get();
  MSKtask_t __mosek_2fusion_2BaseModel__task_get();
  
  void dispose();

  void task_putxx_slice(SolutionType which, int first, int last, std::shared_ptr<monty::ndarray<double,1>> & xx);

  static void env_syeig (int n, std::shared_ptr<monty::ndarray<double,1>> & a, std::shared_ptr<monty::ndarray<double,1>> & w);
  static void env_potrf (int n, std::shared_ptr<monty::ndarray<double,1>> & a);                        
  static void env_syevd (int n, std::shared_ptr<monty::ndarray<double,1>> & a, std::shared_ptr<monty::ndarray<double,1>> & w);

  static void env_putlicensecode(std::shared_ptr<monty::ndarray<int,1>> code);
  static void env_putlicensepath(const std::string &licfile);
  static void env_putlicensewait(int wait);

  static std::string env_getversion();

  void convertSolutionStatus(MSKsoltypee soltype, p_SolutionStruct * sol, MSKsolstae status, MSKprostae prosta);


};

// End of file 'src\fusion\cxx\BaseModel_p.h'
struct p_Model : public ::mosek::fusion::p_BaseModel
{
Model * _pubthis;
static mosek::fusion::p_Model* _get_impl(mosek::fusion::Model * _inst){ return static_cast< mosek::fusion::p_Model* >(mosek::fusion::p_BaseModel::_get_impl(_inst)); }
static mosek::fusion::p_Model * _get_impl(mosek::fusion::Model::t _inst) { return _get_impl(_inst.get()); }
p_Model(Model * _pubthis);
virtual ~p_Model() { /* std::cout << "~p_Model" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::WorkStack > xs{};monty::rc_ptr< ::mosek::fusion::WorkStack > ws{};monty::rc_ptr< ::mosek::fusion::WorkStack > rs{};monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap > con_map{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelConstraint >,1 > > cons{};std::shared_ptr< monty::ndarray< double,1 > > natconmap_objcconst{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_objcode{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_objcodeptr{};std::shared_ptr< monty::ndarray< long long,1 > > natconmap_objcodenidx{};std::shared_ptr< monty::ndarray< double,1 > > natconmap_cconst{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_code{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_codeptr{};std::shared_ptr< monty::ndarray< long long,1 > > natconmap_codenidx{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_coderowptre{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_coderowptrb{};int natconmap_codeatomtop{};int natconmap_codenztop{};int natconmap_numparameterized{};std::shared_ptr< monty::ndarray< double,1 > > param_value{};int param_num{};monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap > par_map{};int numparameter{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > > parameters{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_type{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_idx{};std::shared_ptr< monty::ndarray< long long,1 > > natconmap_slackidx{};std::shared_ptr< monty::ndarray< int,1 > > natconmap_blockid{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natconmap{};std::shared_ptr< monty::ndarray< bool,1 > > initsol_xx_flag{};std::shared_ptr< monty::ndarray< double,1 > > initsol_xx{};monty::rc_ptr< ::mosek::fusion::Utils::StringIntMap > var_map{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelVariable >,1 > > barvars{};std::shared_ptr< monty::ndarray< int,1 > > natbarvarmap_ptr{};std::shared_ptr< monty::ndarray< int,1 > > natbarvarmap_num{};int natbarvarmap_nblock{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_dim{};std::shared_ptr< monty::ndarray< long long,1 > > natbarvar_ptr{};int natbarvar_numbarvarelm{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_j{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_i{};std::shared_ptr< monty::ndarray< int,1 > > natbarvar_idx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_type{};std::shared_ptr< monty::ndarray< int,1 > > natconemap_dim{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natconemap{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::ModelVariable >,1 > > vars{};int bfixidx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_idx{};std::shared_ptr< monty::ndarray< int,1 > > natvarmap_blockid{};monty::rc_ptr< ::mosek::fusion::LinkedBlocks > natvarmap{};mosek::fusion::SolutionType solutionptr{};mosek::fusion::AccSolutionStatus acceptable_sol{};std::string model_name{};virtual void destroy();
static Model::t _new_Model(monty::rc_ptr< ::mosek::fusion::Model > _470);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _470);
static Model::t _new_Model(const std::string &  _476);
void _initialize(const std::string &  _476);
static Model::t _new_Model();
void _initialize();
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2Model__formstConstr(monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _479,std::shared_ptr< monty::ndarray< int,1 > > _480,std::shared_ptr< monty::ndarray< int,1 > > _481) ;
virtual void connames(std::shared_ptr< monty::ndarray< int,1 > > _482,const std::string &  _483,std::shared_ptr< monty::ndarray< int,1 > > _484,std::shared_ptr< monty::ndarray< long long,1 > > _485) ;
virtual void varnames(std::shared_ptr< monty::ndarray< int,1 > > _486,const std::string &  _487,std::shared_ptr< monty::ndarray< int,1 > > _488,std::shared_ptr< monty::ndarray< long long,1 > > _489) ;
virtual void varname(int _490,const std::string &  _491) ;
virtual void natbarvarmap_get(int _492,std::shared_ptr< monty::ndarray< int,1 > > _493) ;
virtual void natbarvar_get(int _497,std::shared_ptr< monty::ndarray< long long,1 > > _498) ;
virtual int natbarvarmap_alloc(int _505,int _506) ;
virtual int natvarmap_alloc(int _524) ;
virtual void natconmap_codealloc(int _534,int _535) ;
virtual int natconmap_alloc(int _542) ;
virtual int natconemap_alloc(int _552) ;
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > _555) ;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > _561) ;
static  void putlicensewait(bool _567);
static  void putlicensepath(const std::string &  _568);
static  void putlicensecode(std::shared_ptr< monty::ndarray< int,1 > > _569);
virtual /* override */ void dispose() ;
virtual void nativeVarToStr(int _574,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _575) ;
virtual MSKtask_t __mosek_2fusion_2Model__getTask() ;
virtual void getConstraintDuals(bool _576,std::shared_ptr< monty::ndarray< int,1 > > _577,std::shared_ptr< monty::ndarray< double,1 > > _578,int _579) ;
virtual void getConstraintValues(bool _589,std::shared_ptr< monty::ndarray< int,1 > > _590,std::shared_ptr< monty::ndarray< double,1 > > _591,int _592) ;
virtual void getVariableDuals(bool _600,std::shared_ptr< monty::ndarray< long long,1 > > _601,std::shared_ptr< monty::ndarray< double,1 > > _602,int _603) ;
virtual void getVariableValues(bool _609,std::shared_ptr< monty::ndarray< long long,1 > > _610,std::shared_ptr< monty::ndarray< double,1 > > _611,int _612) ;
virtual void setVariableValues(bool _618,std::shared_ptr< monty::ndarray< long long,1 > > _619,std::shared_ptr< monty::ndarray< double,1 > > _620) ;
virtual void flushNames() ;
virtual void writeTaskNoFlush(const std::string &  _630) ;
virtual void writeTask(const std::string &  _631) ;
virtual long long getSolverLIntInfo(const std::string &  _632) ;
virtual int getSolverIntInfo(const std::string &  _633) ;
virtual double getSolverDoubleInfo(const std::string &  _634) ;
virtual void setCallbackHandler(mosek::cbhandler_t  _635) ;
virtual void setDataCallbackHandler(mosek::datacbhandler_t  _636) ;
virtual void setLogHandler(mosek::msghandler_t  _637) ;
virtual void setSolverParam(const std::string &  _638,double _639) ;
virtual void setSolverParam(const std::string &  _640,int _641) ;
virtual void setSolverParam(const std::string &  _642,const std::string &  _643) ;
virtual void breakSolver() ;
virtual void optserverHost(const std::string &  _644) ;
virtual void solve(const std::string &  _645,const std::string &  _646) ;
virtual void solve() ;
virtual void flush_parameters() ;
virtual void flushParameters() ;
virtual void evaluate_parameterized(monty::rc_ptr< ::mosek::fusion::WorkStack > _660,int _661,std::shared_ptr< monty::ndarray< int,1 > > _662,std::shared_ptr< monty::ndarray< int,1 > > _663,std::shared_ptr< monty::ndarray< long long,1 > > _664,std::shared_ptr< monty::ndarray< int,1 > > _665,std::shared_ptr< monty::ndarray< int,1 > > _666,std::shared_ptr< monty::ndarray< double,1 > > _667,std::shared_ptr< monty::ndarray< int,1 > > _668,std::shared_ptr< monty::ndarray< double,1 > > _669) ;
virtual void flushSolutions() ;
virtual void flush_initsol(mosek::fusion::SolutionType _679) ;
virtual mosek::fusion::SolutionStatus getDualSolutionStatus() ;
virtual mosek::fusion::ProblemStatus getProblemStatus() ;
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus() ;
virtual double dualObjValue() ;
virtual double primalObjValue() ;
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2Model__get_sol_cache(mosek::fusion::SolutionType _686,bool _687,bool _688) ;
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2Model__get_sol_cache(mosek::fusion::SolutionType _694,bool _695) ;
virtual void setSolution_xx(std::shared_ptr< monty::ndarray< int,1 > > _696,std::shared_ptr< monty::ndarray< double,1 > > _697) ;
virtual void ensure_initsol_xx() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_bars(mosek::fusion::SolutionType _704) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_barx(mosek::fusion::SolutionType _705) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_y(mosek::fusion::SolutionType _706) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_xc(mosek::fusion::SolutionType _707) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_snx(mosek::fusion::SolutionType _708) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_suc(mosek::fusion::SolutionType _709) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_slc(mosek::fusion::SolutionType _710) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_sux(mosek::fusion::SolutionType _711) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_slx(mosek::fusion::SolutionType _712) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_yx(mosek::fusion::SolutionType _713) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getSolution_xx(mosek::fusion::SolutionType _714) ;
virtual void selectedSolution(mosek::fusion::SolutionType _715) ;
virtual mosek::fusion::AccSolutionStatus getAcceptedSolutionStatus() ;
virtual void acceptedSolutionStatus(mosek::fusion::AccSolutionStatus _716) ;
virtual mosek::fusion::ProblemStatus getProblemStatus(mosek::fusion::SolutionType _717) ;
virtual mosek::fusion::SolutionStatus getDualSolutionStatus(mosek::fusion::SolutionType _719) ;
virtual mosek::fusion::SolutionStatus getPrimalSolutionStatus(mosek::fusion::SolutionType _720) ;
virtual mosek::fusion::SolutionStatus getSolutionStatus(mosek::fusion::SolutionType _721,bool _722) ;
virtual void update(std::shared_ptr< monty::ndarray< int,1 > > _725,monty::rc_ptr< ::mosek::fusion::Expression > _726) ;
virtual void update(std::shared_ptr< monty::ndarray< int,1 > > _770,monty::rc_ptr< ::mosek::fusion::Expression > _771,std::shared_ptr< monty::ndarray< int,1 > > _772) ;
virtual void updateObjective(monty::rc_ptr< ::mosek::fusion::Expression > _817,monty::rc_ptr< ::mosek::fusion::Variable > _818) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter_unchecked(const std::string &  _855,std::shared_ptr< monty::ndarray< int,1 > > _856,std::shared_ptr< monty::ndarray< long long,1 > > _857) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter_(const std::string &  _867,std::shared_ptr< monty::ndarray< int,1 > > _868,std::shared_ptr< monty::ndarray< long long,1 > > _869) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter_(const std::string &  _874,std::shared_ptr< monty::ndarray< int,1 > > _875,std::shared_ptr< monty::ndarray< int,2 > > _876) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _884) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _886,int _887,int _888,int _889) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _891,int _892,int _893) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _895,int _896) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _898,std::shared_ptr< monty::ndarray< int,1 > > _899) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _901,std::shared_ptr< monty::ndarray< int,1 > > _902,std::shared_ptr< monty::ndarray< long long,1 > > _903) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(const std::string &  _904,std::shared_ptr< monty::ndarray< int,1 > > _905,std::shared_ptr< monty::ndarray< int,2 > > _906) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter() ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(int _908,int _909,int _910) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(int _912,int _913) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(int _915) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(std::shared_ptr< monty::ndarray< int,1 > > _917) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(std::shared_ptr< monty::ndarray< int,1 > > _919,std::shared_ptr< monty::ndarray< long long,1 > > _920) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__parameter(std::shared_ptr< monty::ndarray< int,1 > > _921,std::shared_ptr< monty::ndarray< int,2 > > _922) ;
virtual void objective_(const std::string &  _923,mosek::fusion::ObjectiveSense _924,monty::rc_ptr< ::mosek::fusion::Expression > _925) ;
virtual void objective(double _972) ;
virtual void objective(mosek::fusion::ObjectiveSense _973,double _974) ;
virtual void objective(mosek::fusion::ObjectiveSense _975,monty::rc_ptr< ::mosek::fusion::Expression > _976) ;
virtual void objective(const std::string &  _977,double _978) ;
virtual void objective(const std::string &  _979,mosek::fusion::ObjectiveSense _980,double _981) ;
virtual void objective(const std::string &  _982,mosek::fusion::ObjectiveSense _983,monty::rc_ptr< ::mosek::fusion::Expression > _984) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _985,monty::rc_ptr< ::mosek::fusion::ConeDomain > _986) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _987,monty::rc_ptr< ::mosek::fusion::Expression > _988,monty::rc_ptr< ::mosek::fusion::ConeDomain > _989) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedConstraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _990,monty::rc_ptr< ::mosek::fusion::RangeDomain > _991) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedConstraint > __mosek_2fusion_2Model__constraint(const std::string &  _992,monty::rc_ptr< ::mosek::fusion::Expression > _993,monty::rc_ptr< ::mosek::fusion::RangeDomain > _994) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _995,monty::rc_ptr< ::mosek::fusion::LinearDomain > _996) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _997,monty::rc_ptr< ::mosek::fusion::Expression > _998,monty::rc_ptr< ::mosek::fusion::LinearDomain > _999) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _1000,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1001) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _1002,monty::rc_ptr< ::mosek::fusion::Expression > _1003,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1004) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(monty::rc_ptr< ::mosek::fusion::Expression > _1005,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1006) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint(const std::string &  _1007,monty::rc_ptr< ::mosek::fusion::Expression > _1008,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1009) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1010) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1011,int _1012,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1013) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1014,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1015) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1016,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1017) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1018,int _1019,int _1020,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1021) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1022,int _1023,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1024) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1025,std::shared_ptr< monty::ndarray< int,1 > > _1026,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1027) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::PSDDomain > _1028) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1029,int _1030,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1031) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1032,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1033) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1034,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1035) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1036,int _1037,int _1038,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1039) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1040,int _1041,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1042) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1043,std::shared_ptr< monty::ndarray< int,1 > > _1044,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1045) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::ConeDomain > _1046) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::RangeDomain > _1047) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(monty::rc_ptr< ::mosek::fusion::LinearDomain > _1048) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _1049,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1050) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _1051,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1052) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _1053,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1054) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(std::shared_ptr< monty::ndarray< int,1 > > _1055) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1056,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1057) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(int _1058,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1059) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1060,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1061) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(int _1062) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1063,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1064) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(const std::string &  _1065,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1066) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1067,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1068) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1069,std::shared_ptr< monty::ndarray< int,1 > > _1070,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1071) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(const std::string &  _1072,std::shared_ptr< monty::ndarray< int,1 > > _1073,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1074) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1075,std::shared_ptr< monty::ndarray< int,1 > > _1076,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1077) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1078,std::shared_ptr< monty::ndarray< int,1 > > _1079) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1080,int _1081,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1082) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__variable(const std::string &  _1083,int _1084,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1085) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1086,int _1087,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1088) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1089,int _1090) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable(const std::string &  _1091) ;
virtual void removeConstraintBlock(int _1092) ;
virtual void removeVariableBlock(long long _1096) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedVariable > __mosek_2fusion_2Model__ranged_variable(const std::string &  _1104,std::shared_ptr< monty::ndarray< int,1 > > _1105,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1106) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _1127,std::shared_ptr< monty::ndarray< int,1 > > _1128,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1129) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _1157,std::shared_ptr< monty::ndarray< int,1 > > _1158,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1159) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__variable_(const std::string &  _1201,std::shared_ptr< monty::ndarray< int,1 > > _1202,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1203) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricVariable > __mosek_2fusion_2Model__variable_(const std::string &  _1224,std::shared_ptr< monty::ndarray< int,1 > > _1225,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1226) ;
virtual void replace_row_code(monty::rc_ptr< ::mosek::fusion::WorkStack > _1241,std::shared_ptr< monty::ndarray< int,1 > > _1242,int _1243,int _1244,int _1245,int _1246,int _1247) ;
virtual monty::rc_ptr< ::mosek::fusion::RangedConstraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1262,monty::rc_ptr< ::mosek::fusion::Expression > _1263,monty::rc_ptr< ::mosek::fusion::RangeDomain > _1264) ;
virtual void putarows(std::shared_ptr< monty::ndarray< int,1 > > _1316,monty::rc_ptr< ::mosek::fusion::WorkStack > _1317,int _1318,int _1319,int _1320,int _1321,int _1322,std::shared_ptr< monty::ndarray< int,1 > > _1323) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1364,monty::rc_ptr< ::mosek::fusion::Expression > _1365,monty::rc_ptr< ::mosek::fusion::PSDDomain > _1366) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1447,monty::rc_ptr< ::mosek::fusion::Expression > _1448,monty::rc_ptr< ::mosek::fusion::LinPSDDomain > _1449) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1514,monty::rc_ptr< ::mosek::fusion::Expression > _1515,monty::rc_ptr< ::mosek::fusion::ConeDomain > _1516) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__constraint_(const std::string &  _1576,monty::rc_ptr< ::mosek::fusion::Expression > _1577,monty::rc_ptr< ::mosek::fusion::LinearDomain > _1578) ;
static  std::string getVersion();
virtual bool hasParameter(const std::string &  _1619) ;
virtual bool hasConstraint(const std::string &  _1620) ;
virtual bool hasVariable(const std::string &  _1621) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Model__getParameter(const std::string &  _1622) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__getConstraint(int _1623) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Model__getConstraint(const std::string &  _1624) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__getVariable(int _1625) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Model__getVariable(const std::string &  _1626) ;
virtual std::string getName() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getParameterValue(std::shared_ptr< monty::ndarray< int,1 > > _1628) ;
virtual void setParameterValue(std::shared_ptr< monty::ndarray< int,1 > > _1631,std::shared_ptr< monty::ndarray< double,1 > > _1632) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Model__clone() ;
}; // struct Model;

// mosek.fusion.Debug from file 'src\fusion\cxx\Debug_p.h'
// namespace mosek::fusion
struct p_Debug
{
  Debug * _pubthis;

  p_Debug(Debug * _pubthis) : _pubthis(_pubthis) {}

  static Debug::t o ()                 { return monty::rc_ptr<Debug>(new Debug()); }
  Debug::t p (const std::string & val) { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (      int val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (long long val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (   double val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t p (     bool val)           { std::cout << val; return Debug::t(_pubthis); }
  Debug::t lf()                        { std::cout << std::endl; return Debug::t(_pubthis); }

  static p_Debug * _get_impl(Debug * _inst) { return _inst->ptr; }

  template<typename T>
  Debug::t p(const std::shared_ptr<monty::ndarray<T,1>> & val)
  {
    if (val->size() > 0)
    {
      std::cout << (*val)[0];
      for (int i = 1; i < val->size(); ++i)
        std::cout << "," << (*val)[i];
    }
    return Debug::t(_pubthis);
  }

  Debug::t __mosek_2fusion_2Debug__p (const std::string & val) { std::cout << val; return Debug::t(_pubthis); }

  template<class C>
  Debug::t __mosek_2fusion_2Debug__p (C val) { std::cout << val; return Debug::t(_pubthis); }
  Debug::t __mosek_2fusion_2Debug__lf()      { std::cout << std::endl; return Debug::t(_pubthis); }
  
};
// End of file 'src\fusion\cxx\Debug_p.h'
struct p_Sort
{
Sort * _pubthis;
static mosek::fusion::p_Sort* _get_impl(mosek::fusion::Sort * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Sort * _get_impl(mosek::fusion::Sort::t _inst) { return _get_impl(_inst.get()); }
p_Sort(Sort * _pubthis);
virtual ~p_Sort() { /* std::cout << "~p_Sort" << std::endl;*/ };
virtual void destroy();
static  void argTransposeSort(std::shared_ptr< monty::ndarray< long long,1 > > _157,std::shared_ptr< monty::ndarray< long long,1 > > _158,int _159,int _160,int _161,std::shared_ptr< monty::ndarray< long long,1 > > _162);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _170,std::shared_ptr< monty::ndarray< long long,1 > > _171);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _172,std::shared_ptr< monty::ndarray< int,1 > > _173);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _174,std::shared_ptr< monty::ndarray< long long,1 > > _175,std::shared_ptr< monty::ndarray< long long,1 > > _176);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _177,std::shared_ptr< monty::ndarray< int,1 > > _178,std::shared_ptr< monty::ndarray< int,1 > > _179);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _180,std::shared_ptr< monty::ndarray< long long,1 > > _181,long long _182,long long _183);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _184,std::shared_ptr< monty::ndarray< int,1 > > _185,long long _186,long long _187);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _188,std::shared_ptr< monty::ndarray< long long,1 > > _189,std::shared_ptr< monty::ndarray< long long,1 > > _190,long long _191,long long _192);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _193,std::shared_ptr< monty::ndarray< int,1 > > _194,std::shared_ptr< monty::ndarray< int,1 > > _195,long long _196,long long _197);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _198,std::shared_ptr< monty::ndarray< long long,1 > > _199,long long _200,long long _201,bool _202);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _205,std::shared_ptr< monty::ndarray< int,1 > > _206,long long _207,long long _208,bool _209);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _212,std::shared_ptr< monty::ndarray< long long,1 > > _213,std::shared_ptr< monty::ndarray< long long,1 > > _214,long long _215,long long _216,bool _217);
static  void argsort(std::shared_ptr< monty::ndarray< long long,1 > > _220,std::shared_ptr< monty::ndarray< int,1 > > _221,std::shared_ptr< monty::ndarray< int,1 > > _222,long long _223,long long _224,bool _225);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > _228,std::shared_ptr< monty::ndarray< long long,1 > > _229,long long _230,long long _231,long long _232,long long _233);
static  void argbucketsort(std::shared_ptr< monty::ndarray< long long,1 > > _234,std::shared_ptr< monty::ndarray< int,1 > > _235,long long _236,long long _237,int _238,int _239);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > _240,std::shared_ptr< monty::ndarray< long long,1 > > _241,std::shared_ptr< monty::ndarray< long long,1 > > _242,long long _243,long long _244,std::shared_ptr< monty::ndarray< long long,1 > > _245);
static  void getminmax(std::shared_ptr< monty::ndarray< long long,1 > > _248,std::shared_ptr< monty::ndarray< int,1 > > _249,std::shared_ptr< monty::ndarray< int,1 > > _250,long long _251,long long _252,std::shared_ptr< monty::ndarray< int,1 > > _253);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _256,std::shared_ptr< monty::ndarray< long long,1 > > _257,long long _258,long long _259,bool _260);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _262,std::shared_ptr< monty::ndarray< int,1 > > _263,long long _264,long long _265,bool _266);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _268,std::shared_ptr< monty::ndarray< long long,1 > > _269,std::shared_ptr< monty::ndarray< long long,1 > > _270,long long _271,long long _272,bool _273);
static  bool issorted(std::shared_ptr< monty::ndarray< long long,1 > > _275,std::shared_ptr< monty::ndarray< int,1 > > _276,std::shared_ptr< monty::ndarray< int,1 > > _277,long long _278,long long _279,bool _280);
}; // struct Sort;

struct p_IndexCounter
{
IndexCounter * _pubthis;
static mosek::fusion::p_IndexCounter* _get_impl(mosek::fusion::IndexCounter * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_IndexCounter * _get_impl(mosek::fusion::IndexCounter::t _inst) { return _get_impl(_inst.get()); }
p_IndexCounter(IndexCounter * _pubthis);
virtual ~p_IndexCounter() { /* std::cout << "~p_IndexCounter" << std::endl;*/ };
long long start{};std::shared_ptr< monty::ndarray< int,1 > > dims{};std::shared_ptr< monty::ndarray< long long,1 > > strides{};std::shared_ptr< monty::ndarray< long long,1 > > st{};std::shared_ptr< monty::ndarray< int,1 > > ii{};int n{};virtual void destroy();
static IndexCounter::t _new_IndexCounter(std::shared_ptr< monty::ndarray< int,1 > > _282);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _282);
static IndexCounter::t _new_IndexCounter(long long _284,std::shared_ptr< monty::ndarray< int,1 > > _285,std::shared_ptr< monty::ndarray< int,1 > > _286);
void _initialize(long long _284,std::shared_ptr< monty::ndarray< int,1 > > _285,std::shared_ptr< monty::ndarray< int,1 > > _286);
static IndexCounter::t _new_IndexCounter(long long _289,std::shared_ptr< monty::ndarray< int,1 > > _290,std::shared_ptr< monty::ndarray< long long,1 > > _291);
void _initialize(long long _289,std::shared_ptr< monty::ndarray< int,1 > > _290,std::shared_ptr< monty::ndarray< long long,1 > > _291);
virtual bool atEnd() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getIndex() ;
virtual long long next() ;
virtual long long get() ;
virtual void inc() ;
virtual void reset() ;
}; // struct IndexCounter;

struct p_CommonTools
{
CommonTools * _pubthis;
static mosek::fusion::p_CommonTools* _get_impl(mosek::fusion::CommonTools * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_CommonTools * _get_impl(mosek::fusion::CommonTools::t _inst) { return _get_impl(_inst.get()); }
p_CommonTools(CommonTools * _pubthis);
virtual ~p_CommonTools() { /* std::cout << "~p_CommonTools" << std::endl;*/ };
virtual void destroy();
static  std::shared_ptr< monty::ndarray< long long,1 > > resize(std::shared_ptr< monty::ndarray< long long,1 > > _297,int _298);
static  std::shared_ptr< monty::ndarray< int,1 > > resize(std::shared_ptr< monty::ndarray< int,1 > > _300,int _301);
static  std::shared_ptr< monty::ndarray< double,1 > > resize(std::shared_ptr< monty::ndarray< double,1 > > _303,int _304);
static  int binarySearch(std::shared_ptr< monty::ndarray< int,1 > > _306,int _307);
static  int binarySearch(std::shared_ptr< monty::ndarray< long long,1 > > _311,long long _312);
static  int binarySearchR(std::shared_ptr< monty::ndarray< long long,1 > > _314,long long _315);
static  int binarySearchL(std::shared_ptr< monty::ndarray< long long,1 > > _319,long long _320);
static  void ndIncr(std::shared_ptr< monty::ndarray< int,1 > > _324,std::shared_ptr< monty::ndarray< int,1 > > _325,std::shared_ptr< monty::ndarray< int,1 > > _326);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > _328,std::shared_ptr< monty::ndarray< int,1 > > _329,std::shared_ptr< monty::ndarray< double,1 > > _330,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > _331,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< long long,1 > >,1 > > _332,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _333,long long _334,int _335,int _336);
static  void transposeTriplets(std::shared_ptr< monty::ndarray< int,1 > > _349,std::shared_ptr< monty::ndarray< int,1 > > _350,std::shared_ptr< monty::ndarray< double,1 > > _351,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _352,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _353,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _354,long long _355,int _356,int _357);
static  void tripletSort(std::shared_ptr< monty::ndarray< int,1 > > _370,std::shared_ptr< monty::ndarray< int,1 > > _371,std::shared_ptr< monty::ndarray< double,1 > > _372,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _373,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< int,1 > >,1 > > _374,std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< double,1 > >,1 > > _375,long long _376,int _377,int _378);
static  void argMSort(std::shared_ptr< monty::ndarray< int,1 > > _404,std::shared_ptr< monty::ndarray< int,1 > > _405);
static  void mergeInto(std::shared_ptr< monty::ndarray< int,1 > > _410,std::shared_ptr< monty::ndarray< int,1 > > _411,std::shared_ptr< monty::ndarray< int,1 > > _412,int _413,int _414,int _415);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > _421,std::shared_ptr< monty::ndarray< long long,1 > > _422,std::shared_ptr< monty::ndarray< long long,1 > > _423,long long _424,long long _425);
static  void argQsort(std::shared_ptr< monty::ndarray< long long,1 > > _426,std::shared_ptr< monty::ndarray< int,1 > > _427,std::shared_ptr< monty::ndarray< int,1 > > _428,long long _429,long long _430);
}; // struct CommonTools;

struct p_SolutionStruct
{
SolutionStruct * _pubthis;
static mosek::fusion::p_SolutionStruct* _get_impl(mosek::fusion::SolutionStruct * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SolutionStruct * _get_impl(mosek::fusion::SolutionStruct::t _inst) { return _get_impl(_inst.get()); }
p_SolutionStruct(SolutionStruct * _pubthis);
virtual ~p_SolutionStruct() { /* std::cout << "~p_SolutionStruct" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > yx{};std::shared_ptr< monty::ndarray< double,1 > > snx{};std::shared_ptr< monty::ndarray< double,1 > > sux{};std::shared_ptr< monty::ndarray< double,1 > > slx{};std::shared_ptr< monty::ndarray< double,1 > > bars{};std::shared_ptr< monty::ndarray< double,1 > > barx{};std::shared_ptr< monty::ndarray< double,1 > > y{};std::shared_ptr< monty::ndarray< double,1 > > suc{};std::shared_ptr< monty::ndarray< double,1 > > slc{};std::shared_ptr< monty::ndarray< double,1 > > xx{};std::shared_ptr< monty::ndarray< double,1 > > xc{};double dobj{};double pobj{};mosek::fusion::ProblemStatus probstatus{};mosek::fusion::SolutionStatus dstatus{};mosek::fusion::SolutionStatus pstatus{};int sol_numbarvar{};int sol_numcone{};int sol_numvar{};int sol_numcon{};virtual void destroy();
static SolutionStruct::t _new_SolutionStruct(int _431,int _432,int _433,int _434);
void _initialize(int _431,int _432,int _433,int _434);
static SolutionStruct::t _new_SolutionStruct(monty::rc_ptr< ::mosek::fusion::SolutionStruct > _435);
void _initialize(monty::rc_ptr< ::mosek::fusion::SolutionStruct > _435);
virtual monty::rc_ptr< ::mosek::fusion::SolutionStruct > __mosek_2fusion_2SolutionStruct__clone() ;
virtual void resize(int _436,int _437,int _438,int _439) ;
virtual bool isDualAcceptable(mosek::fusion::AccSolutionStatus _459) ;
virtual bool isPrimalAcceptable(mosek::fusion::AccSolutionStatus _460) ;
virtual bool isAcceptable(mosek::fusion::SolutionStatus _461,mosek::fusion::AccSolutionStatus _462) ;
}; // struct SolutionStruct;

struct p_ConNZStruct
{
ConNZStruct * _pubthis;
static mosek::fusion::p_ConNZStruct* _get_impl(mosek::fusion::ConNZStruct * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConNZStruct * _get_impl(mosek::fusion::ConNZStruct::t _inst) { return _get_impl(_inst.get()); }
p_ConNZStruct(ConNZStruct * _pubthis);
virtual ~p_ConNZStruct() { /* std::cout << "~p_ConNZStruct" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > barmidx{};std::shared_ptr< monty::ndarray< int,1 > > barsubj{};std::shared_ptr< monty::ndarray< int,1 > > barsubi{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< int,1 > > subj{};std::shared_ptr< monty::ndarray< long long,1 > > ptrb{};virtual void destroy();
static ConNZStruct::t _new_ConNZStruct(std::shared_ptr< monty::ndarray< long long,1 > > _463,std::shared_ptr< monty::ndarray< int,1 > > _464,std::shared_ptr< monty::ndarray< double,1 > > _465,std::shared_ptr< monty::ndarray< double,1 > > _466,std::shared_ptr< monty::ndarray< int,1 > > _467,std::shared_ptr< monty::ndarray< int,1 > > _468,std::shared_ptr< monty::ndarray< int,1 > > _469);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _463,std::shared_ptr< monty::ndarray< int,1 > > _464,std::shared_ptr< monty::ndarray< double,1 > > _465,std::shared_ptr< monty::ndarray< double,1 > > _466,std::shared_ptr< monty::ndarray< int,1 > > _467,std::shared_ptr< monty::ndarray< int,1 > > _468,std::shared_ptr< monty::ndarray< int,1 > > _469);
}; // struct ConNZStruct;

struct p_BaseVariable : public /*implements*/ virtual ::mosek::fusion::Variable
{
BaseVariable * _pubthis;
static mosek::fusion::p_BaseVariable* _get_impl(mosek::fusion::BaseVariable * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_BaseVariable * _get_impl(mosek::fusion::BaseVariable::t _inst) { return _get_impl(_inst.get()); }
p_BaseVariable(BaseVariable * _pubthis);
virtual ~p_BaseVariable() { /* std::cout << "~p_BaseVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};monty::rc_ptr< ::mosek::fusion::Model > model{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static BaseVariable::t _new_BaseVariable(monty::rc_ptr< ::mosek::fusion::BaseVariable > _1903,monty::rc_ptr< ::mosek::fusion::Model > _1904);
void _initialize(monty::rc_ptr< ::mosek::fusion::BaseVariable > _1903,monty::rc_ptr< ::mosek::fusion::Model > _1904);
static BaseVariable::t _new_BaseVariable(monty::rc_ptr< ::mosek::fusion::Model > _1905,std::shared_ptr< monty::ndarray< int,1 > > _1906,std::shared_ptr< monty::ndarray< long long,1 > > _1907,std::shared_ptr< monty::ndarray< long long,1 > > _1908);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1905,std::shared_ptr< monty::ndarray< int,1 > > _1906,std::shared_ptr< monty::ndarray< long long,1 > > _1907,std::shared_ptr< monty::ndarray< long long,1 > > _1908);
virtual /* override */ std::string toString() ;
virtual void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _1911,monty::rc_ptr< ::mosek::fusion::WorkStack > _1912,monty::rc_ptr< ::mosek::fusion::WorkStack > _1913) ;
virtual void remove() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__fromTril(int _1931,int _1932) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__fromTril(int _1965) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__fromTril(int _1965) { return __mosek_2fusion_2BaseVariable__fromTril(_1965); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__tril(int _1966,int _1967) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__tril() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__tril() { return __mosek_2fusion_2BaseVariable__tril(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _2021,int _2022,int _2023) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _2021,int _2022,int _2023) { return __mosek_2fusion_2BaseVariable__reshape(_2021,_2022,_2023); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _2024,int _2025) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _2024,int _2025) { return __mosek_2fusion_2BaseVariable__reshape(_2024,_2025); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(int _2026) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(int _2026) { return __mosek_2fusion_2BaseVariable__reshape(_2026); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__reshape(std::shared_ptr< monty::ndarray< int,1 > > _2027) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__reshape(std::shared_ptr< monty::ndarray< int,1 > > _2027) { return __mosek_2fusion_2BaseVariable__reshape(_2027); }
virtual void setLevel(std::shared_ptr< monty::ndarray< double,1 > > _2031) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2BaseVariable__getModel() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Variable__getModel() { return __mosek_2fusion_2BaseVariable__getModel(); }
virtual int getND() ;
virtual int getDim(int _2034) ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual long long getSize() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > level() ;
virtual void makeContinuous() ;
virtual void makeInteger() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__transpose() { return __mosek_2fusion_2BaseVariable__transpose(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _2055,int _2056,int _2057) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _2055,int _2056,int _2057) { return __mosek_2fusion_2BaseVariable__index(_2055,_2056,_2057); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _2058,int _2059) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _2058,int _2059) { return __mosek_2fusion_2BaseVariable__index(_2058,_2059); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _2060) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(std::shared_ptr< monty::ndarray< int,1 > > _2060) { return __mosek_2fusion_2BaseVariable__index(_2060); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _2063) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _2063) { return __mosek_2fusion_2BaseVariable__index(_2063); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2064,std::shared_ptr< monty::ndarray< int,1 > > _2065,std::shared_ptr< monty::ndarray< int,1 > > _2066) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2064,std::shared_ptr< monty::ndarray< int,1 > > _2065,std::shared_ptr< monty::ndarray< int,1 > > _2066) { return __mosek_2fusion_2BaseVariable__pick(_2064,_2065,_2066); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2069,std::shared_ptr< monty::ndarray< int,1 > > _2070) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2069,std::shared_ptr< monty::ndarray< int,1 > > _2070) { return __mosek_2fusion_2BaseVariable__pick(_2069,_2070); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,2 > > _2073) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,2 > > _2073) { return __mosek_2fusion_2BaseVariable__pick(_2073); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2095) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__pick(std::shared_ptr< monty::ndarray< int,1 > > _2095) { return __mosek_2fusion_2BaseVariable__pick(_2095); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag(int _2106) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__antidiag(int _2106) { return __mosek_2fusion_2BaseVariable__antidiag(_2106); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__antidiag() { return __mosek_2fusion_2BaseVariable__antidiag(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag(int _2107) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__diag(int _2107) { return __mosek_2fusion_2BaseVariable__diag(_2107); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__diag() { return __mosek_2fusion_2BaseVariable__diag(); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__general_diag(std::shared_ptr< monty::ndarray< int,1 > > _2108,std::shared_ptr< monty::ndarray< int,1 > > _2109,int _2110) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _2131,std::shared_ptr< monty::ndarray< int,1 > > _2132) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(std::shared_ptr< monty::ndarray< int,1 > > _2131,std::shared_ptr< monty::ndarray< int,1 > > _2132) { return __mosek_2fusion_2BaseVariable__slice(_2131,_2132); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(int _2166,int _2167) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(int _2166,int _2167) { return __mosek_2fusion_2BaseVariable__slice(_2166,_2167); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseVariable__asExpr() ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Variable__asExpr() { return __mosek_2fusion_2BaseVariable__asExpr(); }
virtual int inst(int _2176,std::shared_ptr< monty::ndarray< long long,1 > > _2177,int _2178,std::shared_ptr< monty::ndarray< long long,1 > > _2179) ;
virtual int numInst() ;
virtual void inst(int _2184,std::shared_ptr< monty::ndarray< long long,1 > > _2185) ;
virtual void set_values(std::shared_ptr< monty::ndarray< double,1 > > _2192,bool _2193) ;
virtual void dual_lu(int _2198,std::shared_ptr< monty::ndarray< double,1 > > _2199,bool _2200) ;
virtual void values(int _2203,std::shared_ptr< monty::ndarray< double,1 > > _2204,bool _2205) ;
virtual void make_continuous() ;
virtual void make_integer() ;
}; // struct BaseVariable;

struct p_SliceVariable : public ::mosek::fusion::p_BaseVariable
{
SliceVariable * _pubthis;
static mosek::fusion::p_SliceVariable* _get_impl(mosek::fusion::SliceVariable * _inst){ return static_cast< mosek::fusion::p_SliceVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_SliceVariable * _get_impl(mosek::fusion::SliceVariable::t _inst) { return _get_impl(_inst.get()); }
p_SliceVariable(SliceVariable * _pubthis);
virtual ~p_SliceVariable() { /* std::cout << "~p_SliceVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};virtual void destroy();
static SliceVariable::t _new_SliceVariable(monty::rc_ptr< ::mosek::fusion::Model > _1634,std::shared_ptr< monty::ndarray< int,1 > > _1635,std::shared_ptr< monty::ndarray< long long,1 > > _1636,std::shared_ptr< monty::ndarray< long long,1 > > _1637);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1634,std::shared_ptr< monty::ndarray< int,1 > > _1635,std::shared_ptr< monty::ndarray< long long,1 > > _1636,std::shared_ptr< monty::ndarray< long long,1 > > _1637);
static SliceVariable::t _new_SliceVariable(monty::rc_ptr< ::mosek::fusion::SliceVariable > _1638);
void _initialize(monty::rc_ptr< ::mosek::fusion::SliceVariable > _1638);
}; // struct SliceVariable;

struct p_BoundInterfaceVariable : public ::mosek::fusion::p_SliceVariable
{
BoundInterfaceVariable * _pubthis;
static mosek::fusion::p_BoundInterfaceVariable* _get_impl(mosek::fusion::BoundInterfaceVariable * _inst){ return static_cast< mosek::fusion::p_BoundInterfaceVariable* >(mosek::fusion::p_SliceVariable::_get_impl(_inst)); }
static mosek::fusion::p_BoundInterfaceVariable * _get_impl(mosek::fusion::BoundInterfaceVariable::t _inst) { return _get_impl(_inst.get()); }
p_BoundInterfaceVariable(BoundInterfaceVariable * _pubthis);
virtual ~p_BoundInterfaceVariable() { /* std::cout << "~p_BoundInterfaceVariable" << std::endl;*/ };
bool islower{};virtual void destroy();
static BoundInterfaceVariable::t _new_BoundInterfaceVariable(monty::rc_ptr< ::mosek::fusion::Model > _1685,std::shared_ptr< monty::ndarray< int,1 > > _1686,std::shared_ptr< monty::ndarray< long long,1 > > _1687,std::shared_ptr< monty::ndarray< long long,1 > > _1688,bool _1689);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1685,std::shared_ptr< monty::ndarray< int,1 > > _1686,std::shared_ptr< monty::ndarray< long long,1 > > _1687,std::shared_ptr< monty::ndarray< long long,1 > > _1688,bool _1689);
static BoundInterfaceVariable::t _new_BoundInterfaceVariable(monty::rc_ptr< ::mosek::fusion::SliceVariable > _1690,bool _1691);
void _initialize(monty::rc_ptr< ::mosek::fusion::SliceVariable > _1690,bool _1691);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__transpose() { return __mosek_2fusion_2BoundInterfaceVariable__transpose(); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1693,std::shared_ptr< monty::ndarray< int,1 > > _1694,std::shared_ptr< monty::ndarray< int,1 > > _1695) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1693,std::shared_ptr< monty::ndarray< int,1 > > _1694,std::shared_ptr< monty::ndarray< int,1 > > _1695) { return __mosek_2fusion_2BoundInterfaceVariable__pick(_1693,_1694,_1695); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1696,std::shared_ptr< monty::ndarray< int,1 > > _1697) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1696,std::shared_ptr< monty::ndarray< int,1 > > _1697) { return __mosek_2fusion_2BoundInterfaceVariable__pick(_1696,_1697); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__pick(std::shared_ptr< monty::ndarray< int,2 > > _1698) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,2 > > _1698) { return __mosek_2fusion_2BoundInterfaceVariable__pick(_1698); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1699) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__pick(std::shared_ptr< monty::ndarray< int,1 > > _1699) { return __mosek_2fusion_2BoundInterfaceVariable__pick(_1699); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__antidiag(int _1700) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag(int _1700) { return __mosek_2fusion_2BoundInterfaceVariable__antidiag(_1700); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__antidiag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__antidiag() { return __mosek_2fusion_2BoundInterfaceVariable__antidiag(); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__diag(int _1701) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag(int _1701) { return __mosek_2fusion_2BoundInterfaceVariable__diag(_1701); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__diag() ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__diag() { return __mosek_2fusion_2BoundInterfaceVariable__diag(); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1702,std::shared_ptr< monty::ndarray< int,1 > > _1703) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1702,std::shared_ptr< monty::ndarray< int,1 > > _1703) { return __mosek_2fusion_2BoundInterfaceVariable__slice(_1702,_1703); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BoundInterfaceVariable__slice(int _1704,int _1705) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(int _1704,int _1705) { return __mosek_2fusion_2BoundInterfaceVariable__slice(_1704,_1705); }
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceVariable > __mosek_2fusion_2BoundInterfaceVariable__from_(monty::rc_ptr< ::mosek::fusion::Variable > _1706) ;
}; // struct BoundInterfaceVariable;

struct p_ModelVariable : public ::mosek::fusion::p_BaseVariable
{
ModelVariable * _pubthis;
static mosek::fusion::p_ModelVariable* _get_impl(mosek::fusion::ModelVariable * _inst){ return static_cast< mosek::fusion::p_ModelVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_ModelVariable * _get_impl(mosek::fusion::ModelVariable::t _inst) { return _get_impl(_inst.get()); }
p_ModelVariable(ModelVariable * _pubthis);
virtual ~p_ModelVariable() { /* std::cout << "~p_ModelVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};long long varid{};std::string name{};virtual void destroy();
static ModelVariable::t _new_ModelVariable(monty::rc_ptr< ::mosek::fusion::ModelVariable > _1866,monty::rc_ptr< ::mosek::fusion::Model > _1867);
void _initialize(monty::rc_ptr< ::mosek::fusion::ModelVariable > _1866,monty::rc_ptr< ::mosek::fusion::Model > _1867);
static ModelVariable::t _new_ModelVariable(monty::rc_ptr< ::mosek::fusion::Model > _1868,const std::string &  _1869,std::shared_ptr< monty::ndarray< int,1 > > _1870,long long _1871,std::shared_ptr< monty::ndarray< long long,1 > > _1872,std::shared_ptr< monty::ndarray< long long,1 > > _1873);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1868,const std::string &  _1869,std::shared_ptr< monty::ndarray< int,1 > > _1870,long long _1871,std::shared_ptr< monty::ndarray< long long,1 > > _1872,std::shared_ptr< monty::ndarray< long long,1 > > _1873);
virtual void flushNames() { throw monty::AbstractClassError("Call to abstract method"); }
virtual void elementName(long long _1874,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1875) ;
virtual /* override */ void remove() ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1876) { throw monty::AbstractClassError("Call to abstract method"); }
}; // struct ModelVariable;

struct p_SymRangedVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
SymRangedVariable * _pubthis;
static mosek::fusion::p_SymRangedVariable* _get_impl(mosek::fusion::SymRangedVariable * _inst){ return static_cast< mosek::fusion::p_SymRangedVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_SymRangedVariable * _get_impl(mosek::fusion::SymRangedVariable::t _inst) { return _get_impl(_inst.get()); }
p_SymRangedVariable(SymRangedVariable * _pubthis);
virtual ~p_SymRangedVariable() { /* std::cout << "~p_SymRangedVariable" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static SymRangedVariable::t _new_SymRangedVariable(monty::rc_ptr< ::mosek::fusion::SymRangedVariable > _1639,monty::rc_ptr< ::mosek::fusion::Model > _1640);
void _initialize(monty::rc_ptr< ::mosek::fusion::SymRangedVariable > _1639,monty::rc_ptr< ::mosek::fusion::Model > _1640);
static SymRangedVariable::t _new_SymRangedVariable(monty::rc_ptr< ::mosek::fusion::Model > _1641,const std::string &  _1642,long long _1643,int _1644,std::shared_ptr< monty::ndarray< long long,1 > > _1645,std::shared_ptr< monty::ndarray< int,1 > > _1646);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1641,const std::string &  _1642,long long _1643,int _1644,std::shared_ptr< monty::ndarray< long long,1 > > _1645,std::shared_ptr< monty::ndarray< int,1 > > _1646);
virtual void dual_u(int _1647,std::shared_ptr< monty::ndarray< double,1 > > _1648) ;
virtual void dual_l(int _1649,std::shared_ptr< monty::ndarray< double,1 > > _1650) ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2SymRangedVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1654) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1654) { return __mosek_2fusion_2SymRangedVariable__clone(_1654); }
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_idxs(int _1655,std::shared_ptr< monty::ndarray< long long,1 > > _1656,std::shared_ptr< monty::ndarray< int,1 > > _1657);
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_sp(int _1673,std::shared_ptr< monty::ndarray< long long,1 > > _1674);
}; // struct SymRangedVariable;

struct p_RangedVariable : public ::mosek::fusion::p_ModelVariable
{
RangedVariable * _pubthis;
static mosek::fusion::p_RangedVariable* _get_impl(mosek::fusion::RangedVariable * _inst){ return static_cast< mosek::fusion::p_RangedVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_RangedVariable * _get_impl(mosek::fusion::RangedVariable::t _inst) { return _get_impl(_inst.get()); }
p_RangedVariable(RangedVariable * _pubthis);
virtual ~p_RangedVariable() { /* std::cout << "~p_RangedVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};bool names_flushed{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};virtual void destroy();
static RangedVariable::t _new_RangedVariable(monty::rc_ptr< ::mosek::fusion::RangedVariable > _1711,monty::rc_ptr< ::mosek::fusion::Model > _1712);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangedVariable > _1711,monty::rc_ptr< ::mosek::fusion::Model > _1712);
static RangedVariable::t _new_RangedVariable(monty::rc_ptr< ::mosek::fusion::Model > _1713,const std::string &  _1714,long long _1715,std::shared_ptr< monty::ndarray< int,1 > > _1716,std::shared_ptr< monty::ndarray< long long,1 > > _1717,std::shared_ptr< monty::ndarray< int,1 > > _1718);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1713,const std::string &  _1714,long long _1715,std::shared_ptr< monty::ndarray< int,1 > > _1716,std::shared_ptr< monty::ndarray< long long,1 > > _1717,std::shared_ptr< monty::ndarray< int,1 > > _1718);
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2RangedVariable__elementDesc(long long _1719,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1720) ;
virtual /* override */ void flushNames() ;
virtual void dual_u(int _1724,std::shared_ptr< monty::ndarray< double,1 > > _1725) ;
virtual void dual_l(int _1726,std::shared_ptr< monty::ndarray< double,1 > > _1727) ;
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceVariable > __mosek_2fusion_2RangedVariable__upperBoundVar() ;
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceVariable > __mosek_2fusion_2RangedVariable__lowerBoundVar() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2RangedVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1730) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1730) { return __mosek_2fusion_2RangedVariable__clone(_1730); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1731);
}; // struct RangedVariable;

struct p_LinearPSDVariable : public ::mosek::fusion::p_ModelVariable
{
LinearPSDVariable * _pubthis;
static mosek::fusion::p_LinearPSDVariable* _get_impl(mosek::fusion::LinearPSDVariable * _inst){ return static_cast< mosek::fusion::p_LinearPSDVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_LinearPSDVariable * _get_impl(mosek::fusion::LinearPSDVariable::t _inst) { return _get_impl(_inst.get()); }
p_LinearPSDVariable(LinearPSDVariable * _pubthis);
virtual ~p_LinearPSDVariable() { /* std::cout << "~p_LinearPSDVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};int varid{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};int conedim{};virtual void destroy();
static LinearPSDVariable::t _new_LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable > _1734,monty::rc_ptr< ::mosek::fusion::Model > _1735);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDVariable > _1734,monty::rc_ptr< ::mosek::fusion::Model > _1735);
static LinearPSDVariable::t _new_LinearPSDVariable(monty::rc_ptr< ::mosek::fusion::Model > _1736,const std::string &  _1737,int _1738,std::shared_ptr< monty::ndarray< int,1 > > _1739,int _1740,std::shared_ptr< monty::ndarray< long long,1 > > _1741);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1736,const std::string &  _1737,int _1738,std::shared_ptr< monty::ndarray< int,1 > > _1739,int _1740,std::shared_ptr< monty::ndarray< long long,1 > > _1741);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual void make_continuous(std::shared_ptr< monty::ndarray< long long,1 > > _1744) ;
virtual void make_integer(std::shared_ptr< monty::ndarray< long long,1 > > _1745) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2LinearPSDVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1746) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1746) { return __mosek_2fusion_2LinearPSDVariable__clone(_1746); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< long long,1 > > _1747);
}; // struct LinearPSDVariable;

struct p_PSDVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
PSDVariable * _pubthis;
static mosek::fusion::p_PSDVariable* _get_impl(mosek::fusion::PSDVariable * _inst){ return static_cast< mosek::fusion::p_PSDVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_PSDVariable * _get_impl(mosek::fusion::PSDVariable::t _inst) { return _get_impl(_inst.get()); }
p_PSDVariable(PSDVariable * _pubthis);
virtual ~p_PSDVariable() { /* std::cout << "~p_PSDVariable" << std::endl;*/ };
int conedim2{};int conedim1{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};std::shared_ptr< monty::ndarray< long long,1 > > nativeidxs{};int varid{};virtual void destroy();
static PSDVariable::t _new_PSDVariable(monty::rc_ptr< ::mosek::fusion::PSDVariable > _1749,monty::rc_ptr< ::mosek::fusion::Model > _1750);
void _initialize(monty::rc_ptr< ::mosek::fusion::PSDVariable > _1749,monty::rc_ptr< ::mosek::fusion::Model > _1750);
static PSDVariable::t _new_PSDVariable(monty::rc_ptr< ::mosek::fusion::Model > _1751,const std::string &  _1752,int _1753,std::shared_ptr< monty::ndarray< int,1 > > _1754,int _1755,int _1756,std::shared_ptr< monty::ndarray< long long,1 > > _1757);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1751,const std::string &  _1752,int _1753,std::shared_ptr< monty::ndarray< int,1 > > _1754,int _1755,int _1756,std::shared_ptr< monty::ndarray< long long,1 > > _1757);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2PSDVariable__elementDesc(long long _1760,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1761) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2PSDVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1762) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1762) { return __mosek_2fusion_2PSDVariable__clone(_1762); }
static  std::shared_ptr< monty::ndarray< long long,1 > > fullnativeidxs(std::shared_ptr< monty::ndarray< int,1 > > _1763,int _1764,int _1765,std::shared_ptr< monty::ndarray< long long,1 > > _1766);
}; // struct PSDVariable;

struct p_SymLinearVariable : public ::mosek::fusion::p_ModelVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
SymLinearVariable * _pubthis;
static mosek::fusion::p_SymLinearVariable* _get_impl(mosek::fusion::SymLinearVariable * _inst){ return static_cast< mosek::fusion::p_SymLinearVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_SymLinearVariable * _get_impl(mosek::fusion::SymLinearVariable::t _inst) { return _get_impl(_inst.get()); }
p_SymLinearVariable(SymLinearVariable * _pubthis);
virtual ~p_SymLinearVariable() { /* std::cout << "~p_SymLinearVariable" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static SymLinearVariable::t _new_SymLinearVariable(monty::rc_ptr< ::mosek::fusion::SymLinearVariable > _1791,monty::rc_ptr< ::mosek::fusion::Model > _1792);
void _initialize(monty::rc_ptr< ::mosek::fusion::SymLinearVariable > _1791,monty::rc_ptr< ::mosek::fusion::Model > _1792);
static SymLinearVariable::t _new_SymLinearVariable(monty::rc_ptr< ::mosek::fusion::Model > _1793,const std::string &  _1794,long long _1795,int _1796,std::shared_ptr< monty::ndarray< long long,1 > > _1797,std::shared_ptr< monty::ndarray< int,1 > > _1798);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1793,const std::string &  _1794,long long _1795,int _1796,std::shared_ptr< monty::ndarray< long long,1 > > _1797,std::shared_ptr< monty::ndarray< int,1 > > _1798);
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2SymLinearVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1802) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1802) { return __mosek_2fusion_2SymLinearVariable__clone(_1802); }
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_idxs(int _1803,std::shared_ptr< monty::ndarray< long long,1 > > _1804,std::shared_ptr< monty::ndarray< int,1 > > _1805);
static  std::shared_ptr< monty::ndarray< long long,1 > > mirror_sp(int _1821,std::shared_ptr< monty::ndarray< long long,1 > > _1822);
}; // struct SymLinearVariable;

struct p_LinearVariable : public ::mosek::fusion::p_ModelVariable
{
LinearVariable * _pubthis;
static mosek::fusion::p_LinearVariable* _get_impl(mosek::fusion::LinearVariable * _inst){ return static_cast< mosek::fusion::p_LinearVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_LinearVariable * _get_impl(mosek::fusion::LinearVariable::t _inst) { return _get_impl(_inst.get()); }
p_LinearVariable(LinearVariable * _pubthis);
virtual ~p_LinearVariable() { /* std::cout << "~p_LinearVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static LinearVariable::t _new_LinearVariable(monty::rc_ptr< ::mosek::fusion::LinearVariable > _1833,monty::rc_ptr< ::mosek::fusion::Model > _1834);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearVariable > _1833,monty::rc_ptr< ::mosek::fusion::Model > _1834);
static LinearVariable::t _new_LinearVariable(monty::rc_ptr< ::mosek::fusion::Model > _1835,const std::string &  _1836,long long _1837,std::shared_ptr< monty::ndarray< int,1 > > _1838,std::shared_ptr< monty::ndarray< long long,1 > > _1839,std::shared_ptr< monty::ndarray< int,1 > > _1840);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1835,const std::string &  _1836,long long _1837,std::shared_ptr< monty::ndarray< int,1 > > _1838,std::shared_ptr< monty::ndarray< long long,1 > > _1839,std::shared_ptr< monty::ndarray< int,1 > > _1840);
virtual /* override */ std::string toString() ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2LinearVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1846) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1846) { return __mosek_2fusion_2LinearVariable__clone(_1846); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1847);
}; // struct LinearVariable;

struct p_ConicVariable : public ::mosek::fusion::p_ModelVariable
{
ConicVariable * _pubthis;
static mosek::fusion::p_ConicVariable* _get_impl(mosek::fusion::ConicVariable * _inst){ return static_cast< mosek::fusion::p_ConicVariable* >(mosek::fusion::p_ModelVariable::_get_impl(_inst)); }
static mosek::fusion::p_ConicVariable * _get_impl(mosek::fusion::ConicVariable::t _inst) { return _get_impl(_inst.get()); }
p_ConicVariable(ConicVariable * _pubthis);
virtual ~p_ConicVariable() { /* std::cout << "~p_ConicVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};bool names_flushed{};int varid{};virtual void destroy();
static ConicVariable::t _new_ConicVariable(monty::rc_ptr< ::mosek::fusion::ConicVariable > _1850,monty::rc_ptr< ::mosek::fusion::Model > _1851);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConicVariable > _1850,monty::rc_ptr< ::mosek::fusion::Model > _1851);
static ConicVariable::t _new_ConicVariable(monty::rc_ptr< ::mosek::fusion::Model > _1852,const std::string &  _1853,int _1854,std::shared_ptr< monty::ndarray< int,1 > > _1855,std::shared_ptr< monty::ndarray< int,1 > > _1856);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _1852,const std::string &  _1853,int _1854,std::shared_ptr< monty::ndarray< int,1 > > _1855,std::shared_ptr< monty::ndarray< int,1 > > _1856);
virtual /* override */ std::string toString() ;
virtual /* override */ void flushNames() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ConicVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1862) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelVariable > __mosek_2fusion_2ModelVariable__clone(monty::rc_ptr< ::mosek::fusion::Model > _1862) { return __mosek_2fusion_2ConicVariable__clone(_1862); }
static  std::shared_ptr< monty::ndarray< long long,1 > > globalNativeIndexes(std::shared_ptr< monty::ndarray< int,1 > > _1863);
}; // struct ConicVariable;

struct p_NilVariable : public ::mosek::fusion::p_BaseVariable, public /*implements*/ virtual ::mosek::fusion::SymmetricVariable
{
NilVariable * _pubthis;
static mosek::fusion::p_NilVariable* _get_impl(mosek::fusion::NilVariable * _inst){ return static_cast< mosek::fusion::p_NilVariable* >(mosek::fusion::p_BaseVariable::_get_impl(_inst)); }
static mosek::fusion::p_NilVariable * _get_impl(mosek::fusion::NilVariable::t _inst) { return _get_impl(_inst.get()); }
p_NilVariable(NilVariable * _pubthis);
virtual ~p_NilVariable() { /* std::cout << "~p_NilVariable" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static NilVariable::t _new_NilVariable(std::shared_ptr< monty::ndarray< int,1 > > _1877);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _1877);
static NilVariable::t _new_NilVariable();
void _initialize();
virtual void flushNames() ;
virtual monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > __mosek_2fusion_2NilVariable__elementDesc(long long _1879,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1880) ;
virtual void elementName(long long _1881,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _1882) ;
virtual /* override */ int numInst() ;
virtual int inst(int _1883,std::shared_ptr< monty::ndarray< long long,1 > > _1884,std::shared_ptr< monty::ndarray< long long,1 > > _1885) ;
virtual /* override */ void inst(int _1886,std::shared_ptr< monty::ndarray< long long,1 > > _1887) ;
virtual /* override */ void set_values(std::shared_ptr< monty::ndarray< double,1 > > _1888,bool _1889) ;
virtual /* override */ void values(int _1890,std::shared_ptr< monty::ndarray< double,1 > > _1891,bool _1892) ;
virtual /* override */ void make_continuous() ;
virtual /* override */ void make_integer() ;
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _1893) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(std::shared_ptr< monty::ndarray< int,1 > > _1893) { return __mosek_2fusion_2NilVariable__index(_1893); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(std::shared_ptr< monty::ndarray< int,1 > > _1893) { return __mosek_2fusion_2NilVariable__index(_1893); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__index(int _1895) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__index(int _1895) { return __mosek_2fusion_2NilVariable__index(_1895); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__index(int _1895) { return __mosek_2fusion_2NilVariable__index(_1895); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1897,std::shared_ptr< monty::ndarray< int,1 > > _1898) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1897,std::shared_ptr< monty::ndarray< int,1 > > _1898) { return __mosek_2fusion_2NilVariable__slice(_1897,_1898); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(std::shared_ptr< monty::ndarray< int,1 > > _1897,std::shared_ptr< monty::ndarray< int,1 > > _1898) { return __mosek_2fusion_2NilVariable__slice(_1897,_1898); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2NilVariable__slice(int _1901,int _1902) ;
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2BaseVariable__slice(int _1901,int _1902) { return __mosek_2fusion_2NilVariable__slice(_1901,_1902); }
virtual monty::rc_ptr< ::mosek::fusion::Variable > __mosek_2fusion_2Variable__slice(int _1901,int _1902) { return __mosek_2fusion_2NilVariable__slice(_1901,_1902); }
}; // struct NilVariable;

struct p_Var
{
Var * _pubthis;
static mosek::fusion::p_Var* _get_impl(mosek::fusion::Var * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Var * _get_impl(mosek::fusion::Var::t _inst) { return _get_impl(_inst.get()); }
p_Var(Var * _pubthis);
virtual ~p_Var() { /* std::cout << "~p_Var" << std::endl;*/ };
virtual void destroy();
static  monty::rc_ptr< ::mosek::fusion::Variable > empty(std::shared_ptr< monty::ndarray< int,1 > > _2248);
static  monty::rc_ptr< ::mosek::fusion::Variable > compress(monty::rc_ptr< ::mosek::fusion::Variable > _2250);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _2258,int _2259);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _2260,int _2261,int _2262);
static  monty::rc_ptr< ::mosek::fusion::Variable > flatten(monty::rc_ptr< ::mosek::fusion::Variable > _2263);
static  monty::rc_ptr< ::mosek::fusion::Variable > reshape(monty::rc_ptr< ::mosek::fusion::Variable > _2264,std::shared_ptr< monty::ndarray< int,1 > > _2265);
static  monty::rc_ptr< ::mosek::fusion::Variable > index_permute_(monty::rc_ptr< ::mosek::fusion::Variable > _2266,std::shared_ptr< monty::ndarray< int,1 > > _2267);
static  monty::rc_ptr< ::mosek::fusion::Variable > hrepeat(monty::rc_ptr< ::mosek::fusion::Variable > _2296,int _2297);
static  monty::rc_ptr< ::mosek::fusion::Variable > vrepeat(monty::rc_ptr< ::mosek::fusion::Variable > _2298,int _2299);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > _2300,int _2301);
static  monty::rc_ptr< ::mosek::fusion::Variable > repeat(monty::rc_ptr< ::mosek::fusion::Variable > _2302,int _2303,int _2304);
static  monty::rc_ptr< ::mosek::fusion::Variable > drepeat(monty::rc_ptr< ::mosek::fusion::Variable > _2305,int _2306,int _2307);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > >,1 > > _2371);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > _2373,monty::rc_ptr< ::mosek::fusion::Variable > _2374,monty::rc_ptr< ::mosek::fusion::Variable > _2375);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(monty::rc_ptr< ::mosek::fusion::Variable > _2376,monty::rc_ptr< ::mosek::fusion::Variable > _2377);
static  monty::rc_ptr< ::mosek::fusion::Variable > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2378);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > _2379,monty::rc_ptr< ::mosek::fusion::Variable > _2380,monty::rc_ptr< ::mosek::fusion::Variable > _2381);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(monty::rc_ptr< ::mosek::fusion::Variable > _2382,monty::rc_ptr< ::mosek::fusion::Variable > _2383);
static  monty::rc_ptr< ::mosek::fusion::Variable > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2384);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > _2385,monty::rc_ptr< ::mosek::fusion::Variable > _2386,monty::rc_ptr< ::mosek::fusion::Variable > _2387,int _2388);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(monty::rc_ptr< ::mosek::fusion::Variable > _2389,monty::rc_ptr< ::mosek::fusion::Variable > _2390,int _2391);
static  monty::rc_ptr< ::mosek::fusion::Variable > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2392,int _2393);
static  monty::rc_ptr< ::mosek::fusion::Variable > promote(monty::rc_ptr< ::mosek::fusion::Variable > _2396,int _2397);
static  monty::rc_ptr< ::mosek::fusion::Variable > dstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _2402,int _2403);
}; // struct Var;

struct p_ConstraintCache
{
ConstraintCache * _pubthis;
static mosek::fusion::p_ConstraintCache* _get_impl(mosek::fusion::ConstraintCache * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConstraintCache * _get_impl(mosek::fusion::ConstraintCache::t _inst) { return _get_impl(_inst.get()); }
p_ConstraintCache(ConstraintCache * _pubthis);
virtual ~p_ConstraintCache() { /* std::cout << "~p_ConstraintCache" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > barmatidx{};std::shared_ptr< monty::ndarray< int,1 > > barsubj{};std::shared_ptr< monty::ndarray< int,1 > > barsubi{};long long nbarnz{};long long nunordered{};std::shared_ptr< monty::ndarray< int,1 > > buffer_subi{};std::shared_ptr< monty::ndarray< int,1 > > buffer_subj{};std::shared_ptr< monty::ndarray< double,1 > > buffer_cof{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< int,1 > > subi{};std::shared_ptr< monty::ndarray< int,1 > > subj{};long long nnz{};int nrows{};virtual void destroy();
static ConstraintCache::t _new_ConstraintCache(monty::rc_ptr< ::mosek::fusion::ConstraintCache > _2526);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConstraintCache > _2526);
static ConstraintCache::t _new_ConstraintCache(std::shared_ptr< monty::ndarray< long long,1 > > _2527,std::shared_ptr< monty::ndarray< double,1 > > _2528,std::shared_ptr< monty::ndarray< int,1 > > _2529,std::shared_ptr< monty::ndarray< double,1 > > _2530,std::shared_ptr< monty::ndarray< int,1 > > _2531,std::shared_ptr< monty::ndarray< int,1 > > _2532,std::shared_ptr< monty::ndarray< int,1 > > _2533);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _2527,std::shared_ptr< monty::ndarray< double,1 > > _2528,std::shared_ptr< monty::ndarray< int,1 > > _2529,std::shared_ptr< monty::ndarray< double,1 > > _2530,std::shared_ptr< monty::ndarray< int,1 > > _2531,std::shared_ptr< monty::ndarray< int,1 > > _2532,std::shared_ptr< monty::ndarray< int,1 > > _2533);
virtual void unchecked_add_fx(std::shared_ptr< monty::ndarray< double,1 > > _2536) ;
virtual long long order_barentries() ;
virtual void add_bar(std::shared_ptr< monty::ndarray< int,1 > > _2546,std::shared_ptr< monty::ndarray< int,1 > > _2547,std::shared_ptr< monty::ndarray< int,1 > > _2548) ;
virtual void unchecked_add_l(std::shared_ptr< monty::ndarray< long long,1 > > _2554,std::shared_ptr< monty::ndarray< int,1 > > _2555,std::shared_ptr< monty::ndarray< double,1 > > _2556,std::shared_ptr< monty::ndarray< double,1 > > _2557) ;
virtual void add(std::shared_ptr< monty::ndarray< long long,1 > > _2566,std::shared_ptr< monty::ndarray< int,1 > > _2567,std::shared_ptr< monty::ndarray< double,1 > > _2568,std::shared_ptr< monty::ndarray< double,1 > > _2569) ;
virtual long long flush(std::shared_ptr< monty::ndarray< int,1 > > _2570,std::shared_ptr< monty::ndarray< int,1 > > _2571,std::shared_ptr< monty::ndarray< double,1 > > _2572,std::shared_ptr< monty::ndarray< double,1 > > _2573) ;
virtual long long numUnsorted() ;
virtual monty::rc_ptr< ::mosek::fusion::ConstraintCache > __mosek_2fusion_2ConstraintCache__clone() ;
}; // struct ConstraintCache;

struct p_Constraint
{
Constraint * _pubthis;
static mosek::fusion::p_Constraint* _get_impl(mosek::fusion::Constraint * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Constraint * _get_impl(mosek::fusion::Constraint::t _inst) { return _get_impl(_inst.get()); }
p_Constraint(Constraint * _pubthis);
virtual ~p_Constraint() { /* std::cout << "~p_Constraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::Model > model{};virtual void destroy();
static Constraint::t _new_Constraint(monty::rc_ptr< ::mosek::fusion::Constraint > _2670,monty::rc_ptr< ::mosek::fusion::Model > _2671);
void _initialize(monty::rc_ptr< ::mosek::fusion::Constraint > _2670,monty::rc_ptr< ::mosek::fusion::Model > _2671);
static Constraint::t _new_Constraint(monty::rc_ptr< ::mosek::fusion::Model > _2672,std::shared_ptr< monty::ndarray< int,1 > > _2673,std::shared_ptr< monty::ndarray< int,1 > > _2674);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2672,std::shared_ptr< monty::ndarray< int,1 > > _2673,std::shared_ptr< monty::ndarray< int,1 > > _2674);
virtual /* override */ std::string toString() ;
virtual void toStringArray(std::shared_ptr< monty::ndarray< long long,1 > > _2675,long long _2676,std::shared_ptr< monty::ndarray< std::string,1 > > _2677) ;
virtual void dual_lu(int _2678,std::shared_ptr< monty::ndarray< double,1 > > _2679,bool _2680) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual std::shared_ptr< monty::ndarray< double,1 > > level() ;
virtual void values(bool _2683,int _2684,std::shared_ptr< monty::ndarray< double,1 > > _2685) ;
virtual void remove() ;
virtual void update(std::shared_ptr< monty::ndarray< double,1 > > _2686) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2687) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2691,monty::rc_ptr< ::mosek::fusion::Variable > _2692,bool _2693) ;
virtual void update(monty::rc_ptr< ::mosek::fusion::Expression > _2713,monty::rc_ptr< ::mosek::fusion::Variable > _2714) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Constraint__get_model() ;
virtual int get_nd() ;
virtual long long size() ;
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2717,int _2718);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > _2719,monty::rc_ptr< ::mosek::fusion::Constraint > _2720,monty::rc_ptr< ::mosek::fusion::Constraint > _2721,int _2722);
static  monty::rc_ptr< ::mosek::fusion::Constraint > stack(monty::rc_ptr< ::mosek::fusion::Constraint > _2723,monty::rc_ptr< ::mosek::fusion::Constraint > _2724,int _2725);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2726);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2727);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2728,monty::rc_ptr< ::mosek::fusion::Constraint > _2729,monty::rc_ptr< ::mosek::fusion::Constraint > _2730);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2731,monty::rc_ptr< ::mosek::fusion::Constraint > _2732,monty::rc_ptr< ::mosek::fusion::Constraint > _2733);
static  monty::rc_ptr< ::mosek::fusion::Constraint > hstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2734,monty::rc_ptr< ::mosek::fusion::Constraint > _2735);
static  monty::rc_ptr< ::mosek::fusion::Constraint > vstack(monty::rc_ptr< ::mosek::fusion::Constraint > _2736,monty::rc_ptr< ::mosek::fusion::Constraint > _2737);
static  monty::rc_ptr< ::mosek::fusion::Constraint > dstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Constraint >,1 > > _2738,int _2739);
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(std::shared_ptr< monty::ndarray< int,1 > > _2790) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(int _2797) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(std::shared_ptr< monty::ndarray< int,1 > > _2798,std::shared_ptr< monty::ndarray< int,1 > > _2799) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(int _2818,int _2819) ;
virtual int getND() ;
virtual int getSize() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Constraint__getModel() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getNativeidxs() ;
}; // struct Constraint;

struct p_SliceConstraint : public ::mosek::fusion::p_Constraint
{
SliceConstraint * _pubthis;
static mosek::fusion::p_SliceConstraint* _get_impl(mosek::fusion::SliceConstraint * _inst){ return static_cast< mosek::fusion::p_SliceConstraint* >(mosek::fusion::p_Constraint::_get_impl(_inst)); }
static mosek::fusion::p_SliceConstraint * _get_impl(mosek::fusion::SliceConstraint::t _inst) { return _get_impl(_inst.get()); }
p_SliceConstraint(SliceConstraint * _pubthis);
virtual ~p_SliceConstraint() { /* std::cout << "~p_SliceConstraint" << std::endl;*/ };
virtual void destroy();
static SliceConstraint::t _new_SliceConstraint(monty::rc_ptr< ::mosek::fusion::SliceConstraint > _2616);
void _initialize(monty::rc_ptr< ::mosek::fusion::SliceConstraint > _2616);
static SliceConstraint::t _new_SliceConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2617,std::shared_ptr< monty::ndarray< int,1 > > _2618,std::shared_ptr< monty::ndarray< int,1 > > _2619);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2617,std::shared_ptr< monty::ndarray< int,1 > > _2618,std::shared_ptr< monty::ndarray< int,1 > > _2619);
virtual /* override */ std::string toString() ;
}; // struct SliceConstraint;

struct p_BoundInterfaceConstraint : public ::mosek::fusion::p_SliceConstraint
{
BoundInterfaceConstraint * _pubthis;
static mosek::fusion::p_BoundInterfaceConstraint* _get_impl(mosek::fusion::BoundInterfaceConstraint * _inst){ return static_cast< mosek::fusion::p_BoundInterfaceConstraint* >(mosek::fusion::p_SliceConstraint::_get_impl(_inst)); }
static mosek::fusion::p_BoundInterfaceConstraint * _get_impl(mosek::fusion::BoundInterfaceConstraint::t _inst) { return _get_impl(_inst.get()); }
p_BoundInterfaceConstraint(BoundInterfaceConstraint * _pubthis);
virtual ~p_BoundInterfaceConstraint() { /* std::cout << "~p_BoundInterfaceConstraint" << std::endl;*/ };
bool islower{};virtual void destroy();
static BoundInterfaceConstraint::t _new_BoundInterfaceConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2596,std::shared_ptr< monty::ndarray< int,1 > > _2597,std::shared_ptr< monty::ndarray< int,1 > > _2598,bool _2599);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2596,std::shared_ptr< monty::ndarray< int,1 > > _2597,std::shared_ptr< monty::ndarray< int,1 > > _2598,bool _2599);
static BoundInterfaceConstraint::t _new_BoundInterfaceConstraint(monty::rc_ptr< ::mosek::fusion::SliceConstraint > _2600,bool _2601);
void _initialize(monty::rc_ptr< ::mosek::fusion::SliceConstraint > _2600,bool _2601);
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > dual() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2BoundInterfaceConstraint__slice(std::shared_ptr< monty::ndarray< int,1 > > _2603,std::shared_ptr< monty::ndarray< int,1 > > _2604) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(std::shared_ptr< monty::ndarray< int,1 > > _2603,std::shared_ptr< monty::ndarray< int,1 > > _2604) { return __mosek_2fusion_2BoundInterfaceConstraint__slice(_2603,_2604); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2BoundInterfaceConstraint__slice(int _2606,int _2607) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__slice(int _2606,int _2607) { return __mosek_2fusion_2BoundInterfaceConstraint__slice(_2606,_2607); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2BoundInterfaceConstraint__index(std::shared_ptr< monty::ndarray< int,1 > > _2609) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(std::shared_ptr< monty::ndarray< int,1 > > _2609) { return __mosek_2fusion_2BoundInterfaceConstraint__index(_2609); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2BoundInterfaceConstraint__index(int _2611) ;
virtual monty::rc_ptr< ::mosek::fusion::Constraint > __mosek_2fusion_2Constraint__index(int _2611) { return __mosek_2fusion_2BoundInterfaceConstraint__index(_2611); }
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceConstraint > __mosek_2fusion_2BoundInterfaceConstraint__from_(monty::rc_ptr< ::mosek::fusion::Constraint > _2613) ;
}; // struct BoundInterfaceConstraint;

struct p_ModelConstraint : public ::mosek::fusion::p_Constraint
{
ModelConstraint * _pubthis;
static mosek::fusion::p_ModelConstraint* _get_impl(mosek::fusion::ModelConstraint * _inst){ return static_cast< mosek::fusion::p_ModelConstraint* >(mosek::fusion::p_Constraint::_get_impl(_inst)); }
static mosek::fusion::p_ModelConstraint * _get_impl(mosek::fusion::ModelConstraint::t _inst) { return _get_impl(_inst.get()); }
p_ModelConstraint(ModelConstraint * _pubthis);
virtual ~p_ModelConstraint() { /* std::cout << "~p_ModelConstraint" << std::endl;*/ };
int conid{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};virtual void destroy();
static ModelConstraint::t _new_ModelConstraint(monty::rc_ptr< ::mosek::fusion::ModelConstraint > _2658,monty::rc_ptr< ::mosek::fusion::Model > _2659);
void _initialize(monty::rc_ptr< ::mosek::fusion::ModelConstraint > _2658,monty::rc_ptr< ::mosek::fusion::Model > _2659);
static ModelConstraint::t _new_ModelConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2660,const std::string &  _2661,std::shared_ptr< monty::ndarray< int,1 > > _2662,std::shared_ptr< monty::ndarray< int,1 > > _2663,int _2664);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2660,const std::string &  _2661,std::shared_ptr< monty::ndarray< int,1 > > _2662,std::shared_ptr< monty::ndarray< int,1 > > _2663,int _2664);
virtual /* override */ std::string toString() ;
virtual void flushNames() ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2669) { throw monty::AbstractClassError("Call to abstract method"); }
virtual /* override */ void remove() ;
}; // struct ModelConstraint;

struct p_LinearPSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
LinearPSDConstraint * _pubthis;
static mosek::fusion::p_LinearPSDConstraint* _get_impl(mosek::fusion::LinearPSDConstraint * _inst){ return static_cast< mosek::fusion::p_LinearPSDConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_LinearPSDConstraint * _get_impl(mosek::fusion::LinearPSDConstraint::t _inst) { return _get_impl(_inst.get()); }
p_LinearPSDConstraint(LinearPSDConstraint * _pubthis);
virtual ~p_LinearPSDConstraint() { /* std::cout << "~p_LinearPSDConstraint" << std::endl;*/ };
int conedim{};std::shared_ptr< monty::ndarray< int,1 > > shape{};int conid{};std::shared_ptr< monty::ndarray< long long,1 > > slackidxs{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};virtual void destroy();
static LinearPSDConstraint::t _new_LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint > _2472,monty::rc_ptr< ::mosek::fusion::Model > _2473);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearPSDConstraint > _2472,monty::rc_ptr< ::mosek::fusion::Model > _2473);
static LinearPSDConstraint::t _new_LinearPSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2474,const std::string &  _2475,int _2476,std::shared_ptr< monty::ndarray< int,1 > > _2477,int _2478,std::shared_ptr< monty::ndarray< int,1 > > _2479,std::shared_ptr< monty::ndarray< long long,1 > > _2480);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2474,const std::string &  _2475,int _2476,std::shared_ptr< monty::ndarray< int,1 > > _2477,int _2478,std::shared_ptr< monty::ndarray< int,1 > > _2479,std::shared_ptr< monty::ndarray< long long,1 > > _2480);
virtual void domainToString(long long _2481,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2482) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2LinearPSDConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2486) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2486) { return __mosek_2fusion_2LinearPSDConstraint__clone(_2486); }
}; // struct LinearPSDConstraint;

struct p_PSDConstraint : public ::mosek::fusion::p_ModelConstraint
{
PSDConstraint * _pubthis;
static mosek::fusion::p_PSDConstraint* _get_impl(mosek::fusion::PSDConstraint * _inst){ return static_cast< mosek::fusion::p_PSDConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_PSDConstraint * _get_impl(mosek::fusion::PSDConstraint::t _inst) { return _get_impl(_inst.get()); }
p_PSDConstraint(PSDConstraint * _pubthis);
virtual ~p_PSDConstraint() { /* std::cout << "~p_PSDConstraint" << std::endl;*/ };
bool names_flushed{};int conedim1{};int conedim0{};std::shared_ptr< monty::ndarray< int,1 > > shape{};std::string name{};std::shared_ptr< monty::ndarray< long long,1 > > slackidxs{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};int conid{};virtual void destroy();
static PSDConstraint::t _new_PSDConstraint(monty::rc_ptr< ::mosek::fusion::PSDConstraint > _2487,monty::rc_ptr< ::mosek::fusion::Model > _2488);
void _initialize(monty::rc_ptr< ::mosek::fusion::PSDConstraint > _2487,monty::rc_ptr< ::mosek::fusion::Model > _2488);
static PSDConstraint::t _new_PSDConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2489,const std::string &  _2490,int _2491,std::shared_ptr< monty::ndarray< int,1 > > _2492,int _2493,int _2494,std::shared_ptr< monty::ndarray< long long,1 > > _2495,std::shared_ptr< monty::ndarray< int,1 > > _2496);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2489,const std::string &  _2490,int _2491,std::shared_ptr< monty::ndarray< int,1 > > _2492,int _2493,int _2494,std::shared_ptr< monty::ndarray< long long,1 > > _2495,std::shared_ptr< monty::ndarray< int,1 > > _2496);
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2PSDConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2497) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2497) { return __mosek_2fusion_2PSDConstraint__clone(_2497); }
static  std::shared_ptr< monty::ndarray< int,1 > > computenidxs(std::shared_ptr< monty::ndarray< int,1 > > _2498,int _2499,int _2500,std::shared_ptr< monty::ndarray< int,1 > > _2501);
}; // struct PSDConstraint;

struct p_RangedConstraint : public ::mosek::fusion::p_ModelConstraint
{
RangedConstraint * _pubthis;
static mosek::fusion::p_RangedConstraint* _get_impl(mosek::fusion::RangedConstraint * _inst){ return static_cast< mosek::fusion::p_RangedConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_RangedConstraint * _get_impl(mosek::fusion::RangedConstraint::t _inst) { return _get_impl(_inst.get()); }
p_RangedConstraint(RangedConstraint * _pubthis);
virtual ~p_RangedConstraint() { /* std::cout << "~p_RangedConstraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static RangedConstraint::t _new_RangedConstraint(monty::rc_ptr< ::mosek::fusion::RangedConstraint > _2621,monty::rc_ptr< ::mosek::fusion::Model > _2622);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangedConstraint > _2621,monty::rc_ptr< ::mosek::fusion::Model > _2622);
static RangedConstraint::t _new_RangedConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2623,const std::string &  _2624,std::shared_ptr< monty::ndarray< int,1 > > _2625,std::shared_ptr< monty::ndarray< int,1 > > _2626,int _2627);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2623,const std::string &  _2624,std::shared_ptr< monty::ndarray< int,1 > > _2625,std::shared_ptr< monty::ndarray< int,1 > > _2626,int _2627);
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceConstraint > __mosek_2fusion_2RangedConstraint__upperBoundCon() ;
virtual monty::rc_ptr< ::mosek::fusion::BoundInterfaceConstraint > __mosek_2fusion_2RangedConstraint__lowerBoundCon() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2RangedConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2628) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2628) { return __mosek_2fusion_2RangedConstraint__clone(_2628); }
}; // struct RangedConstraint;

struct p_ConicConstraint : public ::mosek::fusion::p_ModelConstraint
{
ConicConstraint * _pubthis;
static mosek::fusion::p_ConicConstraint* _get_impl(mosek::fusion::ConicConstraint * _inst){ return static_cast< mosek::fusion::p_ConicConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_ConicConstraint * _get_impl(mosek::fusion::ConicConstraint::t _inst) { return _get_impl(_inst.get()); }
p_ConicConstraint(ConicConstraint * _pubthis);
virtual ~p_ConicConstraint() { /* std::cout << "~p_ConicConstraint" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > nativeslack{};std::shared_ptr< monty::ndarray< int,1 > > nativeidxs{};bool names_flushed{};std::string name{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::ConeDomain > dom{};int conid{};virtual void destroy();
static ConicConstraint::t _new_ConicConstraint(monty::rc_ptr< ::mosek::fusion::ConicConstraint > _2629,monty::rc_ptr< ::mosek::fusion::Model > _2630);
void _initialize(monty::rc_ptr< ::mosek::fusion::ConicConstraint > _2629,monty::rc_ptr< ::mosek::fusion::Model > _2630);
static ConicConstraint::t _new_ConicConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2631,const std::string &  _2632,monty::rc_ptr< ::mosek::fusion::ConeDomain > _2633,std::shared_ptr< monty::ndarray< int,1 > > _2634,int _2635,std::shared_ptr< monty::ndarray< int,1 > > _2636,std::shared_ptr< monty::ndarray< int,1 > > _2637);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2631,const std::string &  _2632,monty::rc_ptr< ::mosek::fusion::ConeDomain > _2633,std::shared_ptr< monty::ndarray< int,1 > > _2634,int _2635,std::shared_ptr< monty::ndarray< int,1 > > _2636,std::shared_ptr< monty::ndarray< int,1 > > _2637);
virtual /* override */ void flushNames() ;
virtual /* override */ std::string toString() ;
virtual void domainToString(long long _2644,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2645) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ConicConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2646) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2646) { return __mosek_2fusion_2ConicConstraint__clone(_2646); }
}; // struct ConicConstraint;

struct p_LinearConstraint : public ::mosek::fusion::p_ModelConstraint
{
LinearConstraint * _pubthis;
static mosek::fusion::p_LinearConstraint* _get_impl(mosek::fusion::LinearConstraint * _inst){ return static_cast< mosek::fusion::p_LinearConstraint* >(mosek::fusion::p_ModelConstraint::_get_impl(_inst)); }
static mosek::fusion::p_LinearConstraint * _get_impl(mosek::fusion::LinearConstraint::t _inst) { return _get_impl(_inst.get()); }
p_LinearConstraint(LinearConstraint * _pubthis);
virtual ~p_LinearConstraint() { /* std::cout << "~p_LinearConstraint" << std::endl;*/ };
std::string name{};int conid{};virtual void destroy();
static LinearConstraint::t _new_LinearConstraint(monty::rc_ptr< ::mosek::fusion::LinearConstraint > _2647,monty::rc_ptr< ::mosek::fusion::Model > _2648);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearConstraint > _2647,monty::rc_ptr< ::mosek::fusion::Model > _2648);
static LinearConstraint::t _new_LinearConstraint(monty::rc_ptr< ::mosek::fusion::Model > _2649,const std::string &  _2650,int _2651,std::shared_ptr< monty::ndarray< int,1 > > _2652,std::shared_ptr< monty::ndarray< int,1 > > _2653);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _2649,const std::string &  _2650,int _2651,std::shared_ptr< monty::ndarray< int,1 > > _2652,std::shared_ptr< monty::ndarray< int,1 > > _2653);
virtual /* override */ std::string toString() ;
virtual void domainToString(long long _2655,monty::rc_ptr< ::mosek::fusion::Utils::StringBuffer > _2656) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2LinearConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2657) ;
virtual monty::rc_ptr< ::mosek::fusion::ModelConstraint > __mosek_2fusion_2ModelConstraint__clone(monty::rc_ptr< ::mosek::fusion::Model > _2657) { return __mosek_2fusion_2LinearConstraint__clone(_2657); }
}; // struct LinearConstraint;

struct p_Set
{
Set * _pubthis;
static mosek::fusion::p_Set* _get_impl(mosek::fusion::Set * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Set * _get_impl(mosek::fusion::Set::t _inst) { return _get_impl(_inst.get()); }
p_Set(Set * _pubthis);
virtual ~p_Set() { /* std::cout << "~p_Set" << std::endl;*/ };
virtual void destroy();
static  long long size(std::shared_ptr< monty::ndarray< int,1 > > _2824);
static  bool match(std::shared_ptr< monty::ndarray< int,1 > > _2827,std::shared_ptr< monty::ndarray< int,1 > > _2828);
static  long long linearidx(std::shared_ptr< monty::ndarray< int,1 > > _2830,std::shared_ptr< monty::ndarray< int,1 > > _2831);
static  std::shared_ptr< monty::ndarray< int,1 > > idxtokey(std::shared_ptr< monty::ndarray< int,1 > > _2834,long long _2835);
static  void idxtokey(std::shared_ptr< monty::ndarray< int,1 > > _2837,long long _2838,std::shared_ptr< monty::ndarray< int,1 > > _2839);
static  std::string indexToString(std::shared_ptr< monty::ndarray< int,1 > > _2843,long long _2844);
static  std::string keyToString(std::shared_ptr< monty::ndarray< int,1 > > _2851);
static  void indexToKey(std::shared_ptr< monty::ndarray< int,1 > > _2854,long long _2855,std::shared_ptr< monty::ndarray< int,1 > > _2856);
static  std::shared_ptr< monty::ndarray< long long,1 > > strides(std::shared_ptr< monty::ndarray< int,1 > > _2860);
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< int,1 > > _2864,std::shared_ptr< monty::ndarray< int,1 > > _2865);
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< int,1 > > _2869);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2871,int _2872,int _2873);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2874,int _2875);
static  std::shared_ptr< monty::ndarray< int,1 > > make(int _2876);
static  std::shared_ptr< monty::ndarray< int,1 > > scalar();
static  std::shared_ptr< monty::ndarray< int,1 > > make(std::shared_ptr< monty::ndarray< std::string,1 > > _2877);
}; // struct Set;

struct p_ConeDomain
{
ConeDomain * _pubthis;
static mosek::fusion::p_ConeDomain* _get_impl(mosek::fusion::ConeDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ConeDomain * _get_impl(mosek::fusion::ConeDomain::t _inst) { return _get_impl(_inst.get()); }
p_ConeDomain(ConeDomain * _pubthis);
virtual ~p_ConeDomain() { /* std::cout << "~p_ConeDomain" << std::endl;*/ };
double alpha{};std::shared_ptr< monty::ndarray< int,1 > > shape{};bool int_flag{};bool axisset{};int axisidx{};mosek::fusion::QConeKey key{};virtual void destroy();
static ConeDomain::t _new_ConeDomain(mosek::fusion::QConeKey _2878,double _2879,std::shared_ptr< monty::ndarray< int,1 > > _2880);
void _initialize(mosek::fusion::QConeKey _2878,double _2879,std::shared_ptr< monty::ndarray< int,1 > > _2880);
static ConeDomain::t _new_ConeDomain(mosek::fusion::QConeKey _2881,std::shared_ptr< monty::ndarray< int,1 > > _2882);
void _initialize(mosek::fusion::QConeKey _2881,std::shared_ptr< monty::ndarray< int,1 > > _2882);
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2883) ;
virtual monty::rc_ptr< ::mosek::fusion::ConeDomain > __mosek_2fusion_2ConeDomain__integral() ;
virtual bool axisIsSet() ;
virtual int getAxis() ;
virtual monty::rc_ptr< ::mosek::fusion::ConeDomain > __mosek_2fusion_2ConeDomain__axis(int _2884) ;
}; // struct ConeDomain;

struct p_LinPSDDomain
{
LinPSDDomain * _pubthis;
static mosek::fusion::p_LinPSDDomain* _get_impl(mosek::fusion::LinPSDDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinPSDDomain * _get_impl(mosek::fusion::LinPSDDomain::t _inst) { return _get_impl(_inst.get()); }
p_LinPSDDomain(LinPSDDomain * _pubthis);
virtual ~p_LinPSDDomain() { /* std::cout << "~p_LinPSDDomain" << std::endl;*/ };
int conedim{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static LinPSDDomain::t _new_LinPSDDomain(std::shared_ptr< monty::ndarray< int,1 > > _2885,int _2886);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2885,int _2886);
static LinPSDDomain::t _new_LinPSDDomain(std::shared_ptr< monty::ndarray< int,1 > > _2887);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _2887);
static LinPSDDomain::t _new_LinPSDDomain();
void _initialize();
}; // struct LinPSDDomain;

struct p_PSDDomain
{
PSDDomain * _pubthis;
static mosek::fusion::p_PSDDomain* _get_impl(mosek::fusion::PSDDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_PSDDomain * _get_impl(mosek::fusion::PSDDomain::t _inst) { return _get_impl(_inst.get()); }
p_PSDDomain(PSDDomain * _pubthis);
virtual ~p_PSDDomain() { /* std::cout << "~p_PSDDomain" << std::endl;*/ };
bool axisIsSet{};int conedim2{};int conedim1{};mosek::fusion::PSDKey key{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2888,std::shared_ptr< monty::ndarray< int,1 > > _2889,int _2890,int _2891);
void _initialize(mosek::fusion::PSDKey _2888,std::shared_ptr< monty::ndarray< int,1 > > _2889,int _2890,int _2891);
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2893,std::shared_ptr< monty::ndarray< int,1 > > _2894);
void _initialize(mosek::fusion::PSDKey _2893,std::shared_ptr< monty::ndarray< int,1 > > _2894);
static PSDDomain::t _new_PSDDomain(mosek::fusion::PSDKey _2895);
void _initialize(mosek::fusion::PSDKey _2895);
virtual monty::rc_ptr< ::mosek::fusion::PSDDomain > __mosek_2fusion_2PSDDomain__axis(int _2896,int _2897) ;
}; // struct PSDDomain;

struct p_RangeDomain
{
RangeDomain * _pubthis;
static mosek::fusion::p_RangeDomain* _get_impl(mosek::fusion::RangeDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_RangeDomain * _get_impl(mosek::fusion::RangeDomain::t _inst) { return _get_impl(_inst.get()); }
p_RangeDomain(RangeDomain * _pubthis);
virtual ~p_RangeDomain() { /* std::cout << "~p_RangeDomain" << std::endl;*/ };
bool cardinal_flag{};bool scalable{};std::shared_ptr< monty::ndarray< double,1 > > ub{};std::shared_ptr< monty::ndarray< double,1 > > lb{};std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool empty{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static RangeDomain::t _new_RangeDomain(bool _2899,std::shared_ptr< monty::ndarray< double,1 > > _2900,std::shared_ptr< monty::ndarray< double,1 > > _2901,std::shared_ptr< monty::ndarray< int,1 > > _2902);
void _initialize(bool _2899,std::shared_ptr< monty::ndarray< double,1 > > _2900,std::shared_ptr< monty::ndarray< double,1 > > _2901,std::shared_ptr< monty::ndarray< int,1 > > _2902);
static RangeDomain::t _new_RangeDomain(bool _2903,std::shared_ptr< monty::ndarray< double,1 > > _2904,std::shared_ptr< monty::ndarray< double,1 > > _2905,std::shared_ptr< monty::ndarray< int,1 > > _2906,std::shared_ptr< monty::ndarray< int,2 > > _2907);
void _initialize(bool _2903,std::shared_ptr< monty::ndarray< double,1 > > _2904,std::shared_ptr< monty::ndarray< double,1 > > _2905,std::shared_ptr< monty::ndarray< int,1 > > _2906,std::shared_ptr< monty::ndarray< int,2 > > _2907);
static RangeDomain::t _new_RangeDomain(bool _2908,std::shared_ptr< monty::ndarray< double,1 > > _2909,std::shared_ptr< monty::ndarray< double,1 > > _2910,std::shared_ptr< monty::ndarray< int,1 > > _2911,std::shared_ptr< monty::ndarray< int,2 > > _2912,int _2913);
void _initialize(bool _2908,std::shared_ptr< monty::ndarray< double,1 > > _2909,std::shared_ptr< monty::ndarray< double,1 > > _2910,std::shared_ptr< monty::ndarray< int,1 > > _2911,std::shared_ptr< monty::ndarray< int,2 > > _2912,int _2913);
static RangeDomain::t _new_RangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2914);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2914);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > __mosek_2fusion_2RangeDomain__symmetric() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2915) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2918) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__sparse() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__integral() ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2920,int _2921,int _2922) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2923,int _2924) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(int _2925) ;
virtual monty::rc_ptr< ::mosek::fusion::RangeDomain > __mosek_2fusion_2RangeDomain__withShape(std::shared_ptr< monty::ndarray< int,1 > > _2926) ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2927) ;
}; // struct RangeDomain;

struct p_SymmetricRangeDomain : public ::mosek::fusion::p_RangeDomain
{
SymmetricRangeDomain * _pubthis;
static mosek::fusion::p_SymmetricRangeDomain* _get_impl(mosek::fusion::SymmetricRangeDomain * _inst){ return static_cast< mosek::fusion::p_SymmetricRangeDomain* >(mosek::fusion::p_RangeDomain::_get_impl(_inst)); }
static mosek::fusion::p_SymmetricRangeDomain * _get_impl(mosek::fusion::SymmetricRangeDomain::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricRangeDomain(SymmetricRangeDomain * _pubthis);
virtual ~p_SymmetricRangeDomain() { /* std::cout << "~p_SymmetricRangeDomain" << std::endl;*/ };
int dim{};virtual void destroy();
static SymmetricRangeDomain::t _new_SymmetricRangeDomain(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2898);
void _initialize(monty::rc_ptr< ::mosek::fusion::RangeDomain > _2898);
}; // struct SymmetricRangeDomain;

struct p_SymmetricLinearDomain
{
SymmetricLinearDomain * _pubthis;
static mosek::fusion::p_SymmetricLinearDomain* _get_impl(mosek::fusion::SymmetricLinearDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricLinearDomain * _get_impl(mosek::fusion::SymmetricLinearDomain::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricLinearDomain(SymmetricLinearDomain * _pubthis);
virtual ~p_SymmetricLinearDomain() { /* std::cout << "~p_SymmetricLinearDomain" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool cardinal_flag{};mosek::fusion::RelationKey key{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::LinearDomain > dom{};int dim{};virtual void destroy();
static SymmetricLinearDomain::t _new_SymmetricLinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2929);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2929);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2930) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2933) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2SymmetricLinearDomain__integral() ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2935) ;
}; // struct SymmetricLinearDomain;

struct p_LinearDomain
{
LinearDomain * _pubthis;
static mosek::fusion::p_LinearDomain* _get_impl(mosek::fusion::LinearDomain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinearDomain * _get_impl(mosek::fusion::LinearDomain::t _inst) { return _get_impl(_inst.get()); }
p_LinearDomain(LinearDomain * _pubthis);
virtual ~p_LinearDomain() { /* std::cout << "~p_LinearDomain" << std::endl;*/ };
bool empty{};bool scalable{};std::shared_ptr< monty::ndarray< int,2 > > sparsity{};bool cardinal_flag{};mosek::fusion::RelationKey key{};std::shared_ptr< monty::ndarray< double,1 > > bnd{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static LinearDomain::t _new_LinearDomain(mosek::fusion::RelationKey _2937,bool _2938,std::shared_ptr< monty::ndarray< double,1 > > _2939,std::shared_ptr< monty::ndarray< int,1 > > _2940);
void _initialize(mosek::fusion::RelationKey _2937,bool _2938,std::shared_ptr< monty::ndarray< double,1 > > _2939,std::shared_ptr< monty::ndarray< int,1 > > _2940);
static LinearDomain::t _new_LinearDomain(mosek::fusion::RelationKey _2941,bool _2942,std::shared_ptr< monty::ndarray< double,1 > > _2943,std::shared_ptr< monty::ndarray< int,1 > > _2944,std::shared_ptr< monty::ndarray< int,2 > > _2945,int _2946);
void _initialize(mosek::fusion::RelationKey _2941,bool _2942,std::shared_ptr< monty::ndarray< double,1 > > _2943,std::shared_ptr< monty::ndarray< int,1 > > _2944,std::shared_ptr< monty::ndarray< int,2 > > _2945,int _2946);
static LinearDomain::t _new_LinearDomain(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2947);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinearDomain > _2947);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > __mosek_2fusion_2LinearDomain__symmetric() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse(std::shared_ptr< monty::ndarray< int,2 > > _2948) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse(std::shared_ptr< monty::ndarray< int,1 > > _2951) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__sparse() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__integral() ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2953,int _2954,int _2955) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2956,int _2957) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(int _2958) ;
virtual monty::rc_ptr< ::mosek::fusion::LinearDomain > __mosek_2fusion_2LinearDomain__withShape(std::shared_ptr< monty::ndarray< int,1 > > _2959) ;
virtual bool match_shape(std::shared_ptr< monty::ndarray< int,1 > > _2960) ;
}; // struct LinearDomain;

struct p_Domain
{
Domain * _pubthis;
static mosek::fusion::p_Domain* _get_impl(mosek::fusion::Domain * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Domain * _get_impl(mosek::fusion::Domain::t _inst) { return _get_impl(_inst.get()); }
p_Domain(Domain * _pubthis);
virtual ~p_Domain() { /* std::cout << "~p_Domain" << std::endl;*/ };
virtual void destroy();
static  long long dimsize(std::shared_ptr< monty::ndarray< int,1 > > _2962);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > mkRangedDomain(monty::rc_ptr< ::mosek::fusion::Matrix > _2965,monty::rc_ptr< ::mosek::fusion::Matrix > _2966);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > mkRangedDomain(std::shared_ptr< monty::ndarray< double,2 > > _2995,std::shared_ptr< monty::ndarray< double,2 > > _2996);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > mkLinearDomain(mosek::fusion::RelationKey _3005,monty::rc_ptr< ::mosek::fusion::Matrix > _3006);
static  long long prod(std::shared_ptr< monty::ndarray< int,1 > > _3012);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(bool _3015,std::shared_ptr< monty::ndarray< double,1 > > _3016,std::shared_ptr< monty::ndarray< double,1 > > _3017,std::shared_ptr< monty::ndarray< int,2 > > _3018,std::shared_ptr< monty::ndarray< int,1 > > _3019);
static  monty::rc_ptr< ::mosek::fusion::SymmetricRangeDomain > symmetric(monty::rc_ptr< ::mosek::fusion::RangeDomain > _3021);
static  monty::rc_ptr< ::mosek::fusion::SymmetricLinearDomain > symmetric(monty::rc_ptr< ::mosek::fusion::LinearDomain > _3022);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse(monty::rc_ptr< ::mosek::fusion::RangeDomain > _3023,std::shared_ptr< monty::ndarray< int,2 > > _3024);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > sparse(monty::rc_ptr< ::mosek::fusion::RangeDomain > _3025,std::shared_ptr< monty::ndarray< int,1 > > _3026);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse(monty::rc_ptr< ::mosek::fusion::LinearDomain > _3027,std::shared_ptr< monty::ndarray< int,2 > > _3028);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > sparse(monty::rc_ptr< ::mosek::fusion::LinearDomain > _3029,std::shared_ptr< monty::ndarray< int,1 > > _3030);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > integral(monty::rc_ptr< ::mosek::fusion::RangeDomain > _3031);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > integral(monty::rc_ptr< ::mosek::fusion::LinearDomain > _3032);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > integral(monty::rc_ptr< ::mosek::fusion::ConeDomain > _3033);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > axis(monty::rc_ptr< ::mosek::fusion::ConeDomain > _3034,int _3035);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _3036,std::shared_ptr< monty::ndarray< int,1 > > _3037);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _3039,int _3040);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDPowerCone(double _3041);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _3042,std::shared_ptr< monty::ndarray< int,1 > > _3043);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _3045,int _3046);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPPowerCone(double _3047);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone(std::shared_ptr< monty::ndarray< int,1 > > _3048);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone(int _3050);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inDExpCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone(std::shared_ptr< monty::ndarray< int,1 > > _3051);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone(int _3053);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inPExpCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(std::shared_ptr< monty::ndarray< int,1 > > _3054);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(int _3056,int _3057);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone(int _3058);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inRotatedQCone();
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(std::shared_ptr< monty::ndarray< int,1 > > _3059);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(int _3061,int _3062);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone(int _3063);
static  monty::rc_ptr< ::mosek::fusion::ConeDomain > inQCone();
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int _3064,int _3065);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD(int _3066);
static  monty::rc_ptr< ::mosek::fusion::LinPSDDomain > isLinPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int _3067,int _3068);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD(int _3069);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > isTrilPSD();
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int _3070,int _3071);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone(int _3072);
static  monty::rc_ptr< ::mosek::fusion::PSDDomain > inPSDCone();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary();
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(std::shared_ptr< monty::ndarray< int,1 > > _3073);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int _3074,int _3075);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > binary(int _3076);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(monty::rc_ptr< ::mosek::fusion::Matrix > _3077,monty::rc_ptr< ::mosek::fusion::Matrix > _3078);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,2 > > _3079,std::shared_ptr< monty::ndarray< double,2 > > _3080);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _3081,std::shared_ptr< monty::ndarray< double,1 > > _3082,std::shared_ptr< monty::ndarray< int,1 > > _3083);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _3084,double _3085,std::shared_ptr< monty::ndarray< int,1 > > _3086);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _3088,std::shared_ptr< monty::ndarray< double,1 > > _3089,std::shared_ptr< monty::ndarray< int,1 > > _3090);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _3092,double _3093,std::shared_ptr< monty::ndarray< int,1 > > _3094);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _3095,std::shared_ptr< monty::ndarray< double,1 > > _3096);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(std::shared_ptr< monty::ndarray< double,1 > > _3097,double _3098);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _3100,std::shared_ptr< monty::ndarray< double,1 > > _3101);
static  monty::rc_ptr< ::mosek::fusion::RangeDomain > inRange(double _3103,double _3104);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(monty::rc_ptr< ::mosek::fusion::Matrix > _3105);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > _3106,std::shared_ptr< monty::ndarray< int,1 > > _3107);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,2 > > _3108);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(std::shared_ptr< monty::ndarray< double,1 > > _3111);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _3112,std::shared_ptr< monty::ndarray< int,1 > > _3113);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _3115,int _3116,int _3117);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _3119,int _3120);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > greaterThan(double _3122);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(monty::rc_ptr< ::mosek::fusion::Matrix > _3123);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > _3124,std::shared_ptr< monty::ndarray< int,1 > > _3125);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,2 > > _3126);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(std::shared_ptr< monty::ndarray< double,1 > > _3129);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _3130,std::shared_ptr< monty::ndarray< int,1 > > _3131);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _3132,int _3133,int _3134);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _3135,int _3136);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > lessThan(double _3137);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(monty::rc_ptr< ::mosek::fusion::Matrix > _3138);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > _3139,std::shared_ptr< monty::ndarray< int,1 > > _3140);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,2 > > _3141);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(std::shared_ptr< monty::ndarray< double,1 > > _3144);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _3145,std::shared_ptr< monty::ndarray< int,1 > > _3146);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _3147,int _3148,int _3149);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _3150,int _3151);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > equalsTo(double _3152);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(std::shared_ptr< monty::ndarray< int,1 > > _3153);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int _3155,int _3156);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded(int _3157);
static  monty::rc_ptr< ::mosek::fusion::LinearDomain > unbounded();
}; // struct Domain;

struct p_ExprCode
{
ExprCode * _pubthis;
static mosek::fusion::p_ExprCode* _get_impl(mosek::fusion::ExprCode * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ExprCode * _get_impl(mosek::fusion::ExprCode::t _inst) { return _get_impl(_inst.get()); }
p_ExprCode(ExprCode * _pubthis);
virtual ~p_ExprCode() { /* std::cout << "~p_ExprCode" << std::endl;*/ };
virtual void destroy();
static  void inplace_relocate(std::shared_ptr< monty::ndarray< int,1 > > _3158,int _3159,int _3160,int _3161);
static  std::string op2str(int _3163);
static  void eval_add_list(std::shared_ptr< monty::ndarray< int,1 > > _3164,std::shared_ptr< monty::ndarray< int,1 > > _3165,std::shared_ptr< monty::ndarray< double,1 > > _3166,int _3167,std::shared_ptr< monty::ndarray< double,1 > > _3168,std::shared_ptr< monty::ndarray< double,1 > > _3169,monty::rc_ptr< ::mosek::fusion::WorkStack > _3170);
static  void eval_add_list(std::shared_ptr< monty::ndarray< int,1 > > _3178,std::shared_ptr< monty::ndarray< int,1 > > _3179,std::shared_ptr< monty::ndarray< double,1 > > _3180,std::shared_ptr< monty::ndarray< double,1 > > _3181,std::shared_ptr< monty::ndarray< double,1 > > _3182,monty::rc_ptr< ::mosek::fusion::WorkStack > _3183);
static  int emit_sum(std::shared_ptr< monty::ndarray< int,1 > > _3184,int _3185,int _3186);
static  int emit_inv(std::shared_ptr< monty::ndarray< int,1 > > _3187,int _3188);
static  int emit_mul(std::shared_ptr< monty::ndarray< int,1 > > _3189,int _3190);
static  int emit_neg(std::shared_ptr< monty::ndarray< int,1 > > _3191,int _3192);
static  int emit_add(std::shared_ptr< monty::ndarray< int,1 > > _3193,int _3194);
static  int emit_constref(std::shared_ptr< monty::ndarray< int,1 > > _3195,int _3196,int _3197);
static  int emit_paramref(std::shared_ptr< monty::ndarray< int,1 > > _3198,int _3199,int _3200);
static  int emit_nop(std::shared_ptr< monty::ndarray< int,1 > > _3201,int _3202);
}; // struct ExprCode;

struct p_Param
{
Param * _pubthis;
static mosek::fusion::p_Param* _get_impl(mosek::fusion::Param * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Param * _get_impl(mosek::fusion::Param::t _inst) { return _get_impl(_inst.get()); }
p_Param(Param * _pubthis);
virtual ~p_Param() { /* std::cout << "~p_Param" << std::endl;*/ };
virtual void destroy();
static  monty::rc_ptr< ::mosek::fusion::Parameter > repeat(monty::rc_ptr< ::mosek::fusion::Parameter > _3211,int _3212,int _3213);
static  monty::rc_ptr< ::mosek::fusion::Parameter > stack(int _3215,monty::rc_ptr< ::mosek::fusion::Parameter > _3216,monty::rc_ptr< ::mosek::fusion::Parameter > _3217,monty::rc_ptr< ::mosek::fusion::Parameter > _3218);
static  monty::rc_ptr< ::mosek::fusion::Parameter > stack(int _3219,monty::rc_ptr< ::mosek::fusion::Parameter > _3220,monty::rc_ptr< ::mosek::fusion::Parameter > _3221);
static  monty::rc_ptr< ::mosek::fusion::Parameter > stack(int _3222,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > > _3223);
static  monty::rc_ptr< ::mosek::fusion::Parameter > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > >,1 > > _3224);
static  monty::rc_ptr< ::mosek::fusion::Parameter > hstack(monty::rc_ptr< ::mosek::fusion::Parameter > _3226,monty::rc_ptr< ::mosek::fusion::Parameter > _3227,monty::rc_ptr< ::mosek::fusion::Parameter > _3228);
static  monty::rc_ptr< ::mosek::fusion::Parameter > hstack(monty::rc_ptr< ::mosek::fusion::Parameter > _3229,monty::rc_ptr< ::mosek::fusion::Parameter > _3230);
static  monty::rc_ptr< ::mosek::fusion::Parameter > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > > _3231);
static  monty::rc_ptr< ::mosek::fusion::Parameter > vstack(monty::rc_ptr< ::mosek::fusion::Parameter > _3232,monty::rc_ptr< ::mosek::fusion::Parameter > _3233,monty::rc_ptr< ::mosek::fusion::Parameter > _3234);
static  monty::rc_ptr< ::mosek::fusion::Parameter > vstack(monty::rc_ptr< ::mosek::fusion::Parameter > _3235,monty::rc_ptr< ::mosek::fusion::Parameter > _3236);
static  monty::rc_ptr< ::mosek::fusion::Parameter > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > > _3237);
static  monty::rc_ptr< ::mosek::fusion::Parameter > dstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Parameter >,1 > > _3238,int _3239);
}; // struct Param;

struct p_ParameterImpl : public /*implements*/ virtual ::mosek::fusion::Parameter
{
ParameterImpl * _pubthis;
static mosek::fusion::p_ParameterImpl* _get_impl(mosek::fusion::ParameterImpl * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_ParameterImpl * _get_impl(mosek::fusion::ParameterImpl::t _inst) { return _get_impl(_inst.get()); }
p_ParameterImpl(ParameterImpl * _pubthis);
virtual ~p_ParameterImpl() { /* std::cout << "~p_ParameterImpl" << std::endl;*/ };
long long size{};std::shared_ptr< monty::ndarray< int,1 > > nidxs{};std::shared_ptr< monty::ndarray< long long,1 > > sp{};std::shared_ptr< monty::ndarray< int,1 > > shape{};monty::rc_ptr< ::mosek::fusion::Model > model{};virtual void destroy();
static ParameterImpl::t _new_ParameterImpl(monty::rc_ptr< ::mosek::fusion::ParameterImpl > _4028,monty::rc_ptr< ::mosek::fusion::Model > _4029);
void _initialize(monty::rc_ptr< ::mosek::fusion::ParameterImpl > _4028,monty::rc_ptr< ::mosek::fusion::Model > _4029);
static ParameterImpl::t _new_ParameterImpl(monty::rc_ptr< ::mosek::fusion::Model > _4030,std::shared_ptr< monty::ndarray< int,1 > > _4031,std::shared_ptr< monty::ndarray< long long,1 > > _4032,std::shared_ptr< monty::ndarray< int,1 > > _4033);
void _initialize(monty::rc_ptr< ::mosek::fusion::Model > _4030,std::shared_ptr< monty::ndarray< int,1 > > _4031,std::shared_ptr< monty::ndarray< long long,1 > > _4032,std::shared_ptr< monty::ndarray< int,1 > > _4033);
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2ParameterImpl__clone(monty::rc_ptr< ::mosek::fusion::Model > _4034) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Parameter__clone(monty::rc_ptr< ::mosek::fusion::Model > _4034) { return __mosek_2fusion_2ParameterImpl__clone(_4034); }
virtual /* override */ std::string toString() ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ParameterImpl__pick(std::shared_ptr< monty::ndarray< int,2 > > _4037) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,2 > > _4037) { return __mosek_2fusion_2ParameterImpl__pick(_4037); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ParameterImpl__pick(std::shared_ptr< monty::ndarray< int,1 > > _4038) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,1 > > _4038) { return __mosek_2fusion_2ParameterImpl__pick(_4038); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ParameterImpl__index(std::shared_ptr< monty::ndarray< int,1 > > _4039) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(std::shared_ptr< monty::ndarray< int,1 > > _4039) { return __mosek_2fusion_2ParameterImpl__index(_4039); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ParameterImpl__index(int _4048) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(int _4048) { return __mosek_2fusion_2ParameterImpl__index(_4048); }
virtual void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4050,monty::rc_ptr< ::mosek::fusion::WorkStack > _4051,monty::rc_ptr< ::mosek::fusion::WorkStack > _4052) ;
virtual void getSp(std::shared_ptr< monty::ndarray< long long,1 > > _4074,int _4075) ;
virtual bool isSparse() ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2ParameterImpl__slice(std::shared_ptr< monty::ndarray< int,1 > > _4078,std::shared_ptr< monty::ndarray< int,1 > > _4079) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Parameter__slice(std::shared_ptr< monty::ndarray< int,1 > > _4078,std::shared_ptr< monty::ndarray< int,1 > > _4079) { return __mosek_2fusion_2ParameterImpl__slice(_4078,_4079); }
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2ParameterImpl__slice(int _4111,int _4112) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Parameter__slice(int _4111,int _4112) { return __mosek_2fusion_2ParameterImpl__slice(_4111,_4112); }
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2ParameterImpl__reshape(std::shared_ptr< monty::ndarray< int,1 > > _4120) ;
virtual monty::rc_ptr< ::mosek::fusion::Parameter > __mosek_2fusion_2Parameter__reshape(std::shared_ptr< monty::ndarray< int,1 > > _4120) { return __mosek_2fusion_2ParameterImpl__reshape(_4120); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ParameterImpl__asExpr() ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Parameter__asExpr() { return __mosek_2fusion_2ParameterImpl__asExpr(); }
virtual long long getSize() ;
virtual int getNumNonzero() ;
virtual int getND() ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual int getDim(int _4121) ;
virtual void getAllIndexes(std::shared_ptr< monty::ndarray< int,1 > > _4122,int _4123) ;
virtual int getIndex(int _4125) ;
virtual std::shared_ptr< monty::ndarray< double,1 > > getValue() ;
virtual void setValue(std::shared_ptr< monty::ndarray< double,2 > > _4126) ;
virtual void setValue(std::shared_ptr< monty::ndarray< double,1 > > _4132) ;
virtual void setValue(double _4135) ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2ParameterImpl__getModel() ;
virtual monty::rc_ptr< ::mosek::fusion::Model > __mosek_2fusion_2Parameter__getModel() { return __mosek_2fusion_2ParameterImpl__getModel(); }
}; // struct ParameterImpl;

struct p_BaseExpression : public /*implements*/ virtual ::mosek::fusion::Expression
{
BaseExpression * _pubthis;
static mosek::fusion::p_BaseExpression* _get_impl(mosek::fusion::BaseExpression * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_BaseExpression * _get_impl(mosek::fusion::BaseExpression::t _inst) { return _get_impl(_inst.get()); }
p_BaseExpression(BaseExpression * _pubthis);
virtual ~p_BaseExpression() { /* std::cout << "~p_BaseExpression" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static BaseExpression::t _new_BaseExpression(std::shared_ptr< monty::ndarray< int,1 > > _6759);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _6759);
virtual /* override */ std::string toString() ;
virtual void printStack(monty::rc_ptr< ::mosek::fusion::WorkStack > _6760) ;
virtual void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6786,monty::rc_ptr< ::mosek::fusion::WorkStack > _6787,monty::rc_ptr< ::mosek::fusion::WorkStack > _6788) { throw monty::AbstractClassError("Call to abstract method"); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__pick(std::shared_ptr< monty::ndarray< int,2 > > _6789) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,2 > > _6789) { return __mosek_2fusion_2BaseExpression__pick(_6789); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__pick(std::shared_ptr< monty::ndarray< int,1 > > _6790) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__pick(std::shared_ptr< monty::ndarray< int,1 > > _6790) { return __mosek_2fusion_2BaseExpression__pick(_6790); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__index(std::shared_ptr< monty::ndarray< int,1 > > _6793) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(std::shared_ptr< monty::ndarray< int,1 > > _6793) { return __mosek_2fusion_2BaseExpression__index(_6793); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__index(int _6796) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__index(int _6796) { return __mosek_2fusion_2BaseExpression__index(_6796); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(std::shared_ptr< monty::ndarray< int,1 > > _6798,std::shared_ptr< monty::ndarray< int,1 > > _6799) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__slice(std::shared_ptr< monty::ndarray< int,1 > > _6798,std::shared_ptr< monty::ndarray< int,1 > > _6799) { return __mosek_2fusion_2BaseExpression__slice(_6798,_6799); }
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(int _6800,int _6801) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2Expression__slice(int _6800,int _6801) { return __mosek_2fusion_2BaseExpression__slice(_6800,_6801); }
virtual long long getSize() ;
virtual int getND() ;
virtual int getDim(int _6802) ;
virtual std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
}; // struct BaseExpression;

struct p_ExprParameter : public ::mosek::fusion::p_BaseExpression
{
ExprParameter * _pubthis;
static mosek::fusion::p_ExprParameter* _get_impl(mosek::fusion::ExprParameter * _inst){ return static_cast< mosek::fusion::p_ExprParameter* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprParameter * _get_impl(mosek::fusion::ExprParameter::t _inst) { return _get_impl(_inst.get()); }
p_ExprParameter(ExprParameter * _pubthis);
virtual ~p_ExprParameter() { /* std::cout << "~p_ExprParameter" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprParameter::t _new_ExprParameter(monty::rc_ptr< ::mosek::fusion::Parameter > _3203);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3203);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3204,monty::rc_ptr< ::mosek::fusion::WorkStack > _3205,monty::rc_ptr< ::mosek::fusion::WorkStack > _3206) ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ExprParameter__slice(std::shared_ptr< monty::ndarray< int,1 > > _3207,std::shared_ptr< monty::ndarray< int,1 > > _3208) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(std::shared_ptr< monty::ndarray< int,1 > > _3207,std::shared_ptr< monty::ndarray< int,1 > > _3208) { return __mosek_2fusion_2ExprParameter__slice(_3207,_3208); }
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2ExprParameter__slice(int _3209,int _3210) ;
virtual monty::rc_ptr< ::mosek::fusion::Expression > __mosek_2fusion_2BaseExpression__slice(int _3209,int _3210) { return __mosek_2fusion_2ExprParameter__slice(_3209,_3210); }
virtual /* override */ std::string toString() ;
}; // struct ExprParameter;

struct p_ExprMulParamScalarExpr : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamScalarExpr * _pubthis;
static mosek::fusion::p_ExprMulParamScalarExpr* _get_impl(mosek::fusion::ExprMulParamScalarExpr * _inst){ return static_cast< mosek::fusion::p_ExprMulParamScalarExpr* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamScalarExpr * _get_impl(mosek::fusion::ExprMulParamScalarExpr::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamScalarExpr(ExprMulParamScalarExpr * _pubthis);
virtual ~p_ExprMulParamScalarExpr() { /* std::cout << "~p_ExprMulParamScalarExpr" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamScalarExpr::t _new_ExprMulParamScalarExpr(monty::rc_ptr< ::mosek::fusion::Parameter > _3298,monty::rc_ptr< ::mosek::fusion::Expression > _3299);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3298,monty::rc_ptr< ::mosek::fusion::Expression > _3299);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3300,monty::rc_ptr< ::mosek::fusion::WorkStack > _3301,monty::rc_ptr< ::mosek::fusion::WorkStack > _3302) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamScalarExpr;

struct p_ExprMulParamScalar : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamScalar * _pubthis;
static mosek::fusion::p_ExprMulParamScalar* _get_impl(mosek::fusion::ExprMulParamScalar * _inst){ return static_cast< mosek::fusion::p_ExprMulParamScalar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamScalar * _get_impl(mosek::fusion::ExprMulParamScalar::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamScalar(ExprMulParamScalar * _pubthis);
virtual ~p_ExprMulParamScalar() { /* std::cout << "~p_ExprMulParamScalar" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamScalar::t _new_ExprMulParamScalar(monty::rc_ptr< ::mosek::fusion::Parameter > _3353,monty::rc_ptr< ::mosek::fusion::Expression > _3354);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3353,monty::rc_ptr< ::mosek::fusion::Expression > _3354);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3355,monty::rc_ptr< ::mosek::fusion::WorkStack > _3356,monty::rc_ptr< ::mosek::fusion::WorkStack > _3357) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamScalar;

struct p_ExprMulParamDiagLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamDiagLeft * _pubthis;
static mosek::fusion::p_ExprMulParamDiagLeft* _get_impl(mosek::fusion::ExprMulParamDiagLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulParamDiagLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamDiagLeft * _get_impl(mosek::fusion::ExprMulParamDiagLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamDiagLeft(ExprMulParamDiagLeft * _pubthis);
virtual ~p_ExprMulParamDiagLeft() { /* std::cout << "~p_ExprMulParamDiagLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamDiagLeft::t _new_ExprMulParamDiagLeft(monty::rc_ptr< ::mosek::fusion::Parameter > _3400,monty::rc_ptr< ::mosek::fusion::Expression > _3401);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3400,monty::rc_ptr< ::mosek::fusion::Expression > _3401);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3402,monty::rc_ptr< ::mosek::fusion::WorkStack > _3403,monty::rc_ptr< ::mosek::fusion::WorkStack > _3404) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamDiagLeft;

struct p_ExprMulParamDiagRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamDiagRight * _pubthis;
static mosek::fusion::p_ExprMulParamDiagRight* _get_impl(mosek::fusion::ExprMulParamDiagRight * _inst){ return static_cast< mosek::fusion::p_ExprMulParamDiagRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamDiagRight * _get_impl(mosek::fusion::ExprMulParamDiagRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamDiagRight(ExprMulParamDiagRight * _pubthis);
virtual ~p_ExprMulParamDiagRight() { /* std::cout << "~p_ExprMulParamDiagRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamDiagRight::t _new_ExprMulParamDiagRight(monty::rc_ptr< ::mosek::fusion::Expression > _3519,monty::rc_ptr< ::mosek::fusion::Parameter > _3520);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _3519,monty::rc_ptr< ::mosek::fusion::Parameter > _3520);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3521,monty::rc_ptr< ::mosek::fusion::WorkStack > _3522,monty::rc_ptr< ::mosek::fusion::WorkStack > _3523) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamDiagRight;

struct p_ExprDotParam : public ::mosek::fusion::p_BaseExpression
{
ExprDotParam * _pubthis;
static mosek::fusion::p_ExprDotParam* _get_impl(mosek::fusion::ExprDotParam * _inst){ return static_cast< mosek::fusion::p_ExprDotParam* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprDotParam * _get_impl(mosek::fusion::ExprDotParam::t _inst) { return _get_impl(_inst.get()); }
p_ExprDotParam(ExprDotParam * _pubthis);
virtual ~p_ExprDotParam() { /* std::cout << "~p_ExprDotParam" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprDotParam::t _new_ExprDotParam(monty::rc_ptr< ::mosek::fusion::Parameter > _3637,monty::rc_ptr< ::mosek::fusion::Expression > _3638);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3637,monty::rc_ptr< ::mosek::fusion::Expression > _3638);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3640,monty::rc_ptr< ::mosek::fusion::WorkStack > _3641,monty::rc_ptr< ::mosek::fusion::WorkStack > _3642) ;
virtual /* override */ std::string toString() ;
}; // struct ExprDotParam;

struct p_ExprMulParamElem : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamElem * _pubthis;
static mosek::fusion::p_ExprMulParamElem* _get_impl(mosek::fusion::ExprMulParamElem * _inst){ return static_cast< mosek::fusion::p_ExprMulParamElem* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamElem * _get_impl(mosek::fusion::ExprMulParamElem::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamElem(ExprMulParamElem * _pubthis);
virtual ~p_ExprMulParamElem() { /* std::cout << "~p_ExprMulParamElem" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamElem::t _new_ExprMulParamElem(monty::rc_ptr< ::mosek::fusion::Parameter > _3700,monty::rc_ptr< ::mosek::fusion::Expression > _3701);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3700,monty::rc_ptr< ::mosek::fusion::Expression > _3701);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3703,monty::rc_ptr< ::mosek::fusion::WorkStack > _3704,monty::rc_ptr< ::mosek::fusion::WorkStack > _3705) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamElem;

struct p_ExprMulParamRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamRight * _pubthis;
static mosek::fusion::p_ExprMulParamRight* _get_impl(mosek::fusion::ExprMulParamRight * _inst){ return static_cast< mosek::fusion::p_ExprMulParamRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamRight * _get_impl(mosek::fusion::ExprMulParamRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamRight(ExprMulParamRight * _pubthis);
virtual ~p_ExprMulParamRight() { /* std::cout << "~p_ExprMulParamRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamRight::t _new_ExprMulParamRight(monty::rc_ptr< ::mosek::fusion::Expression > _3767,monty::rc_ptr< ::mosek::fusion::Parameter > _3768);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _3767,monty::rc_ptr< ::mosek::fusion::Parameter > _3768);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3769,monty::rc_ptr< ::mosek::fusion::WorkStack > _3770,monty::rc_ptr< ::mosek::fusion::WorkStack > _3771) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamRight;

struct p_ExprMulParamLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulParamLeft * _pubthis;
static mosek::fusion::p_ExprMulParamLeft* _get_impl(mosek::fusion::ExprMulParamLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulParamLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulParamLeft * _get_impl(mosek::fusion::ExprMulParamLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulParamLeft(ExprMulParamLeft * _pubthis);
virtual ~p_ExprMulParamLeft() { /* std::cout << "~p_ExprMulParamLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};monty::rc_ptr< ::mosek::fusion::Parameter > p{};virtual void destroy();
static ExprMulParamLeft::t _new_ExprMulParamLeft(monty::rc_ptr< ::mosek::fusion::Parameter > _3871,monty::rc_ptr< ::mosek::fusion::Expression > _3872);
void _initialize(monty::rc_ptr< ::mosek::fusion::Parameter > _3871,monty::rc_ptr< ::mosek::fusion::Expression > _3872);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _3873,monty::rc_ptr< ::mosek::fusion::WorkStack > _3874,monty::rc_ptr< ::mosek::fusion::WorkStack > _3875) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulParamLeft;

struct p_ExprOptimizeCode : public ::mosek::fusion::p_BaseExpression
{
ExprOptimizeCode * _pubthis;
static mosek::fusion::p_ExprOptimizeCode* _get_impl(mosek::fusion::ExprOptimizeCode * _inst){ return static_cast< mosek::fusion::p_ExprOptimizeCode* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprOptimizeCode * _get_impl(mosek::fusion::ExprOptimizeCode::t _inst) { return _get_impl(_inst.get()); }
p_ExprOptimizeCode(ExprOptimizeCode * _pubthis);
virtual ~p_ExprOptimizeCode() { /* std::cout << "~p_ExprOptimizeCode" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprOptimizeCode::t _new_ExprOptimizeCode(monty::rc_ptr< ::mosek::fusion::Expression > _4153);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4153);
static  void compress_code(monty::rc_ptr< ::mosek::fusion::WorkStack > _4154,int _4155,std::shared_ptr< monty::ndarray< int,1 > > _4156,int _4157,std::shared_ptr< monty::ndarray< int,1 > > _4158,int _4159,std::shared_ptr< monty::ndarray< double,1 > > _4160,int _4161,std::shared_ptr< monty::ndarray< double,1 > > _4162,int _4163,int _4164,int _4165,int _4166);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4219,monty::rc_ptr< ::mosek::fusion::WorkStack > _4220,monty::rc_ptr< ::mosek::fusion::WorkStack > _4221) ;
virtual /* override */ std::string toString() ;
}; // struct ExprOptimizeCode;

struct p_ExprCompress : public ::mosek::fusion::p_BaseExpression
{
ExprCompress * _pubthis;
static mosek::fusion::p_ExprCompress* _get_impl(mosek::fusion::ExprCompress * _inst){ return static_cast< mosek::fusion::p_ExprCompress* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprCompress * _get_impl(mosek::fusion::ExprCompress::t _inst) { return _get_impl(_inst.get()); }
p_ExprCompress(ExprCompress * _pubthis);
virtual ~p_ExprCompress() { /* std::cout << "~p_ExprCompress" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprCompress::t _new_ExprCompress(monty::rc_ptr< ::mosek::fusion::Expression > _4288);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4288);
static  void arg_sort(monty::rc_ptr< ::mosek::fusion::WorkStack > _4289,monty::rc_ptr< ::mosek::fusion::WorkStack > _4290,int _4291,int _4292,int _4293,int _4294,int _4295);
static  void merge_sort(int _4331,int _4332,int _4333,int _4334,int _4335,int _4336,std::shared_ptr< monty::ndarray< int,1 > > _4337,std::shared_ptr< monty::ndarray< long long,1 > > _4338);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4361,monty::rc_ptr< ::mosek::fusion::WorkStack > _4362,monty::rc_ptr< ::mosek::fusion::WorkStack > _4363) ;
virtual /* override */ std::string toString() ;
}; // struct ExprCompress;

struct p_ExprConst : public ::mosek::fusion::p_BaseExpression
{
ExprConst * _pubthis;
static mosek::fusion::p_ExprConst* _get_impl(mosek::fusion::ExprConst * _inst){ return static_cast< mosek::fusion::p_ExprConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprConst * _get_impl(mosek::fusion::ExprConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprConst(ExprConst * _pubthis);
virtual ~p_ExprConst() { /* std::cout << "~p_ExprConst" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > sparsity{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};virtual void destroy();
static ExprConst::t _new_ExprConst(std::shared_ptr< monty::ndarray< int,1 > > _4449,std::shared_ptr< monty::ndarray< long long,1 > > _4450,std::shared_ptr< monty::ndarray< double,1 > > _4451);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4449,std::shared_ptr< monty::ndarray< long long,1 > > _4450,std::shared_ptr< monty::ndarray< double,1 > > _4451);
static ExprConst::t _new_ExprConst(std::shared_ptr< monty::ndarray< int,1 > > _4452,std::shared_ptr< monty::ndarray< long long,1 > > _4453,double _4454);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4452,std::shared_ptr< monty::ndarray< long long,1 > > _4453,double _4454);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4457,monty::rc_ptr< ::mosek::fusion::WorkStack > _4458,monty::rc_ptr< ::mosek::fusion::WorkStack > _4459) ;
static  void validate(std::shared_ptr< monty::ndarray< int,1 > > _4478,std::shared_ptr< monty::ndarray< double,1 > > _4479,std::shared_ptr< monty::ndarray< long long,1 > > _4480);
virtual /* override */ std::string toString() ;
}; // struct ExprConst;

struct p_ExprPick : public ::mosek::fusion::p_BaseExpression
{
ExprPick * _pubthis;
static mosek::fusion::p_ExprPick* _get_impl(mosek::fusion::ExprPick * _inst){ return static_cast< mosek::fusion::p_ExprPick* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprPick * _get_impl(mosek::fusion::ExprPick::t _inst) { return _get_impl(_inst.get()); }
p_ExprPick(ExprPick * _pubthis);
virtual ~p_ExprPick() { /* std::cout << "~p_ExprPick" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > idxs{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprPick::t _new_ExprPick(monty::rc_ptr< ::mosek::fusion::Expression > _4484,std::shared_ptr< monty::ndarray< int,2 > > _4485);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4484,std::shared_ptr< monty::ndarray< int,2 > > _4485);
static ExprPick::t _new_ExprPick(monty::rc_ptr< ::mosek::fusion::Expression > _4497,std::shared_ptr< monty::ndarray< long long,1 > > _4498);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4497,std::shared_ptr< monty::ndarray< long long,1 > > _4498);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4503,monty::rc_ptr< ::mosek::fusion::WorkStack > _4504,monty::rc_ptr< ::mosek::fusion::WorkStack > _4505) ;
virtual /* override */ std::string toString() ;
}; // struct ExprPick;

struct p_ExprSlice : public ::mosek::fusion::p_BaseExpression
{
ExprSlice * _pubthis;
static mosek::fusion::p_ExprSlice* _get_impl(mosek::fusion::ExprSlice * _inst){ return static_cast< mosek::fusion::p_ExprSlice* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSlice * _get_impl(mosek::fusion::ExprSlice::t _inst) { return _get_impl(_inst.get()); }
p_ExprSlice(ExprSlice * _pubthis);
virtual ~p_ExprSlice() { /* std::cout << "~p_ExprSlice" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > last{};std::shared_ptr< monty::ndarray< int,1 > > first{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSlice::t _new_ExprSlice(monty::rc_ptr< ::mosek::fusion::Expression > _4570,std::shared_ptr< monty::ndarray< int,1 > > _4571,std::shared_ptr< monty::ndarray< int,1 > > _4572);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4570,std::shared_ptr< monty::ndarray< int,1 > > _4571,std::shared_ptr< monty::ndarray< int,1 > > _4572);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4573,monty::rc_ptr< ::mosek::fusion::WorkStack > _4574,monty::rc_ptr< ::mosek::fusion::WorkStack > _4575) ;
static  std::shared_ptr< monty::ndarray< int,1 > > makeShape(std::shared_ptr< monty::ndarray< int,1 > > _4640,std::shared_ptr< monty::ndarray< int,1 > > _4641,std::shared_ptr< monty::ndarray< int,1 > > _4642);
virtual /* override */ std::string toString() ;
}; // struct ExprSlice;

struct p_ExprPermuteDims : public ::mosek::fusion::p_BaseExpression
{
ExprPermuteDims * _pubthis;
static mosek::fusion::p_ExprPermuteDims* _get_impl(mosek::fusion::ExprPermuteDims * _inst){ return static_cast< mosek::fusion::p_ExprPermuteDims* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprPermuteDims * _get_impl(mosek::fusion::ExprPermuteDims::t _inst) { return _get_impl(_inst.get()); }
p_ExprPermuteDims(ExprPermuteDims * _pubthis);
virtual ~p_ExprPermuteDims() { /* std::cout << "~p_ExprPermuteDims" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > dperm{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprPermuteDims::t _new_ExprPermuteDims(std::shared_ptr< monty::ndarray< int,1 > > _4647,monty::rc_ptr< ::mosek::fusion::Expression > _4648);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4647,monty::rc_ptr< ::mosek::fusion::Expression > _4648);
static ExprPermuteDims::t _new_ExprPermuteDims(std::shared_ptr< monty::ndarray< int,1 > > _4654,monty::rc_ptr< ::mosek::fusion::Expression > _4655,int _4656);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _4654,monty::rc_ptr< ::mosek::fusion::Expression > _4655,int _4656);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4657,monty::rc_ptr< ::mosek::fusion::WorkStack > _4658,monty::rc_ptr< ::mosek::fusion::WorkStack > _4659) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(std::shared_ptr< monty::ndarray< int,1 > > _4713,std::shared_ptr< monty::ndarray< int,1 > > _4714);
}; // struct ExprPermuteDims;

struct p_ExprTranspose : public ::mosek::fusion::p_BaseExpression
{
ExprTranspose * _pubthis;
static mosek::fusion::p_ExprTranspose* _get_impl(mosek::fusion::ExprTranspose * _inst){ return static_cast< mosek::fusion::p_ExprTranspose* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprTranspose * _get_impl(mosek::fusion::ExprTranspose::t _inst) { return _get_impl(_inst.get()); }
p_ExprTranspose(ExprTranspose * _pubthis);
virtual ~p_ExprTranspose() { /* std::cout << "~p_ExprTranspose" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprTranspose::t _new_ExprTranspose(monty::rc_ptr< ::mosek::fusion::Expression > _4716);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4716);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4717,monty::rc_ptr< ::mosek::fusion::WorkStack > _4718,monty::rc_ptr< ::mosek::fusion::WorkStack > _4719) ;
virtual /* override */ std::string toString() ;
static  std::shared_ptr< monty::ndarray< int,1 > > transposeShape(std::shared_ptr< monty::ndarray< int,1 > > _4772);
}; // struct ExprTranspose;

struct p_ExprRepeat : public ::mosek::fusion::p_BaseExpression
{
ExprRepeat * _pubthis;
static mosek::fusion::p_ExprRepeat* _get_impl(mosek::fusion::ExprRepeat * _inst){ return static_cast< mosek::fusion::p_ExprRepeat* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprRepeat * _get_impl(mosek::fusion::ExprRepeat::t _inst) { return _get_impl(_inst.get()); }
p_ExprRepeat(ExprRepeat * _pubthis);
virtual ~p_ExprRepeat() { /* std::cout << "~p_ExprRepeat" << std::endl;*/ };
int n{};int dim{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprRepeat::t _new_ExprRepeat(monty::rc_ptr< ::mosek::fusion::Expression > _4773,int _4774,int _4775);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _4773,int _4774,int _4775);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4776,monty::rc_ptr< ::mosek::fusion::WorkStack > _4777,monty::rc_ptr< ::mosek::fusion::WorkStack > _4778) ;
static  std::shared_ptr< monty::ndarray< int,1 > > getshape(monty::rc_ptr< ::mosek::fusion::Expression > _4843,int _4844,int _4845);
virtual /* override */ std::string toString() ;
}; // struct ExprRepeat;

struct p_ExprStack : public ::mosek::fusion::p_BaseExpression
{
ExprStack * _pubthis;
static mosek::fusion::p_ExprStack* _get_impl(mosek::fusion::ExprStack * _inst){ return static_cast< mosek::fusion::p_ExprStack* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprStack * _get_impl(mosek::fusion::ExprStack::t _inst) { return _get_impl(_inst.get()); }
p_ExprStack(ExprStack * _pubthis);
virtual ~p_ExprStack() { /* std::cout << "~p_ExprStack" << std::endl;*/ };
int dim{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > exprs{};virtual void destroy();
static ExprStack::t _new_ExprStack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _4850,int _4851);
void _initialize(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _4850,int _4851);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _4853,monty::rc_ptr< ::mosek::fusion::WorkStack > _4854,monty::rc_ptr< ::mosek::fusion::WorkStack > _4855) ;
static  std::shared_ptr< monty::ndarray< int,1 > > getshape(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _4999,int _5000);
virtual /* override */ std::string toString() ;
}; // struct ExprStack;

struct p_ExprInner : public ::mosek::fusion::p_BaseExpression
{
ExprInner * _pubthis;
static mosek::fusion::p_ExprInner* _get_impl(mosek::fusion::ExprInner * _inst){ return static_cast< mosek::fusion::p_ExprInner* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprInner * _get_impl(mosek::fusion::ExprInner::t _inst) { return _get_impl(_inst.get()); }
p_ExprInner(ExprInner * _pubthis);
virtual ~p_ExprInner() { /* std::cout << "~p_ExprInner" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > vcof{};std::shared_ptr< monty::ndarray< long long,1 > > vsub{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _5014,std::shared_ptr< monty::ndarray< long long,1 > > _5015,std::shared_ptr< monty::ndarray< double,1 > > _5016);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _5014,std::shared_ptr< monty::ndarray< long long,1 > > _5015,std::shared_ptr< monty::ndarray< double,1 > > _5016);
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _5022,std::shared_ptr< monty::ndarray< double,1 > > _5023);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _5022,std::shared_ptr< monty::ndarray< double,1 > > _5023);
static ExprInner::t _new_ExprInner(monty::rc_ptr< ::mosek::fusion::Expression > _5025,std::shared_ptr< monty::ndarray< int,2 > > _5026,std::shared_ptr< monty::ndarray< double,1 > > _5027);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _5025,std::shared_ptr< monty::ndarray< int,2 > > _5026,std::shared_ptr< monty::ndarray< double,1 > > _5027);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5028,monty::rc_ptr< ::mosek::fusion::WorkStack > _5029,monty::rc_ptr< ::mosek::fusion::WorkStack > _5030) ;
static  std::shared_ptr< monty::ndarray< long long,1 > > range(int _5074);
static  std::shared_ptr< monty::ndarray< long long,1 > > convert(std::shared_ptr< monty::ndarray< int,1 > > _5076,std::shared_ptr< monty::ndarray< int,2 > > _5077);
virtual /* override */ std::string toString() ;
}; // struct ExprInner;

struct p_ExprMulDiagRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulDiagRight * _pubthis;
static mosek::fusion::p_ExprMulDiagRight* _get_impl(mosek::fusion::ExprMulDiagRight * _inst){ return static_cast< mosek::fusion::p_ExprMulDiagRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulDiagRight * _get_impl(mosek::fusion::ExprMulDiagRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulDiagRight(ExprMulDiagRight * _pubthis);
virtual ~p_ExprMulDiagRight() { /* std::cout << "~p_ExprMulDiagRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulDiagRight::t _new_ExprMulDiagRight(int _5084,int _5085,std::shared_ptr< monty::ndarray< int,1 > > _5086,std::shared_ptr< monty::ndarray< int,1 > > _5087,std::shared_ptr< monty::ndarray< double,1 > > _5088,monty::rc_ptr< ::mosek::fusion::Expression > _5089,int _5090);
void _initialize(int _5084,int _5085,std::shared_ptr< monty::ndarray< int,1 > > _5086,std::shared_ptr< monty::ndarray< int,1 > > _5087,std::shared_ptr< monty::ndarray< double,1 > > _5088,monty::rc_ptr< ::mosek::fusion::Expression > _5089,int _5090);
static ExprMulDiagRight::t _new_ExprMulDiagRight(int _5091,int _5092,std::shared_ptr< monty::ndarray< int,1 > > _5093,std::shared_ptr< monty::ndarray< int,1 > > _5094,std::shared_ptr< monty::ndarray< double,1 > > _5095,monty::rc_ptr< ::mosek::fusion::Expression > _5096);
void _initialize(int _5091,int _5092,std::shared_ptr< monty::ndarray< int,1 > > _5093,std::shared_ptr< monty::ndarray< int,1 > > _5094,std::shared_ptr< monty::ndarray< double,1 > > _5095,monty::rc_ptr< ::mosek::fusion::Expression > _5096);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5097,monty::rc_ptr< ::mosek::fusion::WorkStack > _5098,monty::rc_ptr< ::mosek::fusion::WorkStack > _5099) ;
static  int validate(int _5178,int _5179,std::shared_ptr< monty::ndarray< int,1 > > _5180,std::shared_ptr< monty::ndarray< int,1 > > _5181,std::shared_ptr< monty::ndarray< double,1 > > _5182,monty::rc_ptr< ::mosek::fusion::Expression > _5183);
virtual /* override */ std::string toString() ;
}; // struct ExprMulDiagRight;

struct p_ExprMulDiagLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulDiagLeft * _pubthis;
static mosek::fusion::p_ExprMulDiagLeft* _get_impl(mosek::fusion::ExprMulDiagLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulDiagLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulDiagLeft * _get_impl(mosek::fusion::ExprMulDiagLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulDiagLeft(ExprMulDiagLeft * _pubthis);
virtual ~p_ExprMulDiagLeft() { /* std::cout << "~p_ExprMulDiagLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulDiagLeft::t _new_ExprMulDiagLeft(int _5192,int _5193,std::shared_ptr< monty::ndarray< int,1 > > _5194,std::shared_ptr< monty::ndarray< int,1 > > _5195,std::shared_ptr< monty::ndarray< double,1 > > _5196,monty::rc_ptr< ::mosek::fusion::Expression > _5197,int _5198);
void _initialize(int _5192,int _5193,std::shared_ptr< monty::ndarray< int,1 > > _5194,std::shared_ptr< monty::ndarray< int,1 > > _5195,std::shared_ptr< monty::ndarray< double,1 > > _5196,monty::rc_ptr< ::mosek::fusion::Expression > _5197,int _5198);
static ExprMulDiagLeft::t _new_ExprMulDiagLeft(int _5199,int _5200,std::shared_ptr< monty::ndarray< int,1 > > _5201,std::shared_ptr< monty::ndarray< int,1 > > _5202,std::shared_ptr< monty::ndarray< double,1 > > _5203,monty::rc_ptr< ::mosek::fusion::Expression > _5204);
void _initialize(int _5199,int _5200,std::shared_ptr< monty::ndarray< int,1 > > _5201,std::shared_ptr< monty::ndarray< int,1 > > _5202,std::shared_ptr< monty::ndarray< double,1 > > _5203,monty::rc_ptr< ::mosek::fusion::Expression > _5204);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5205,monty::rc_ptr< ::mosek::fusion::WorkStack > _5206,monty::rc_ptr< ::mosek::fusion::WorkStack > _5207) ;
static  int validate(int _5305,int _5306,std::shared_ptr< monty::ndarray< int,1 > > _5307,std::shared_ptr< monty::ndarray< int,1 > > _5308,std::shared_ptr< monty::ndarray< double,1 > > _5309,monty::rc_ptr< ::mosek::fusion::Expression > _5310);
virtual /* override */ std::string toString() ;
}; // struct ExprMulDiagLeft;

struct p_ExprMulElement : public ::mosek::fusion::p_BaseExpression
{
ExprMulElement * _pubthis;
static mosek::fusion::p_ExprMulElement* _get_impl(mosek::fusion::ExprMulElement * _inst){ return static_cast< mosek::fusion::p_ExprMulElement* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulElement * _get_impl(mosek::fusion::ExprMulElement::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulElement(ExprMulElement * _pubthis);
virtual ~p_ExprMulElement() { /* std::cout << "~p_ExprMulElement" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< long long,1 > > msp{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};virtual void destroy();
static ExprMulElement::t _new_ExprMulElement(std::shared_ptr< monty::ndarray< double,1 > > _5319,std::shared_ptr< monty::ndarray< long long,1 > > _5320,monty::rc_ptr< ::mosek::fusion::Expression > _5321);
void _initialize(std::shared_ptr< monty::ndarray< double,1 > > _5319,std::shared_ptr< monty::ndarray< long long,1 > > _5320,monty::rc_ptr< ::mosek::fusion::Expression > _5321);
static ExprMulElement::t _new_ExprMulElement(std::shared_ptr< monty::ndarray< double,1 > > _5328,std::shared_ptr< monty::ndarray< long long,1 > > _5329,monty::rc_ptr< ::mosek::fusion::Expression > _5330,int _5331);
void _initialize(std::shared_ptr< monty::ndarray< double,1 > > _5328,std::shared_ptr< monty::ndarray< long long,1 > > _5329,monty::rc_ptr< ::mosek::fusion::Expression > _5330,int _5331);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5332,monty::rc_ptr< ::mosek::fusion::WorkStack > _5333,monty::rc_ptr< ::mosek::fusion::WorkStack > _5334) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulElement;

struct p_ExprMulScalarConst : public ::mosek::fusion::p_BaseExpression
{
ExprMulScalarConst * _pubthis;
static mosek::fusion::p_ExprMulScalarConst* _get_impl(mosek::fusion::ExprMulScalarConst * _inst){ return static_cast< mosek::fusion::p_ExprMulScalarConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulScalarConst * _get_impl(mosek::fusion::ExprMulScalarConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulScalarConst(ExprMulScalarConst * _pubthis);
virtual ~p_ExprMulScalarConst() { /* std::cout << "~p_ExprMulScalarConst" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};double c{};virtual void destroy();
static ExprMulScalarConst::t _new_ExprMulScalarConst(double _5392,monty::rc_ptr< ::mosek::fusion::Expression > _5393);
void _initialize(double _5392,monty::rc_ptr< ::mosek::fusion::Expression > _5393);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5394,monty::rc_ptr< ::mosek::fusion::WorkStack > _5395,monty::rc_ptr< ::mosek::fusion::WorkStack > _5396) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulScalarConst;

struct p_ExprScalarMul : public ::mosek::fusion::p_BaseExpression
{
ExprScalarMul * _pubthis;
static mosek::fusion::p_ExprScalarMul* _get_impl(mosek::fusion::ExprScalarMul * _inst){ return static_cast< mosek::fusion::p_ExprScalarMul* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprScalarMul * _get_impl(mosek::fusion::ExprScalarMul::t _inst) { return _get_impl(_inst.get()); }
p_ExprScalarMul(ExprScalarMul * _pubthis);
virtual ~p_ExprScalarMul() { /* std::cout << "~p_ExprScalarMul" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprScalarMul::t _new_ExprScalarMul(int _5434,int _5435,std::shared_ptr< monty::ndarray< int,1 > > _5436,std::shared_ptr< monty::ndarray< int,1 > > _5437,std::shared_ptr< monty::ndarray< double,1 > > _5438,monty::rc_ptr< ::mosek::fusion::Expression > _5439,int _5440);
void _initialize(int _5434,int _5435,std::shared_ptr< monty::ndarray< int,1 > > _5436,std::shared_ptr< monty::ndarray< int,1 > > _5437,std::shared_ptr< monty::ndarray< double,1 > > _5438,monty::rc_ptr< ::mosek::fusion::Expression > _5439,int _5440);
static ExprScalarMul::t _new_ExprScalarMul(int _5441,int _5442,std::shared_ptr< monty::ndarray< int,1 > > _5443,std::shared_ptr< monty::ndarray< int,1 > > _5444,std::shared_ptr< monty::ndarray< double,1 > > _5445,monty::rc_ptr< ::mosek::fusion::Expression > _5446);
void _initialize(int _5441,int _5442,std::shared_ptr< monty::ndarray< int,1 > > _5443,std::shared_ptr< monty::ndarray< int,1 > > _5444,std::shared_ptr< monty::ndarray< double,1 > > _5445,monty::rc_ptr< ::mosek::fusion::Expression > _5446);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5447,monty::rc_ptr< ::mosek::fusion::WorkStack > _5448,monty::rc_ptr< ::mosek::fusion::WorkStack > _5449) ;
static  int validate(int _5485,int _5486,std::shared_ptr< monty::ndarray< int,1 > > _5487,std::shared_ptr< monty::ndarray< int,1 > > _5488,std::shared_ptr< monty::ndarray< double,1 > > _5489,monty::rc_ptr< ::mosek::fusion::Expression > _5490);
virtual /* override */ std::string toString() ;
}; // struct ExprScalarMul;

struct p_ExprMulRight : public ::mosek::fusion::p_BaseExpression
{
ExprMulRight * _pubthis;
static mosek::fusion::p_ExprMulRight* _get_impl(mosek::fusion::ExprMulRight * _inst){ return static_cast< mosek::fusion::p_ExprMulRight* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulRight * _get_impl(mosek::fusion::ExprMulRight::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulRight(ExprMulRight * _pubthis);
virtual ~p_ExprMulRight() { /* std::cout << "~p_ExprMulRight" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulRight::t _new_ExprMulRight(int _5497,int _5498,std::shared_ptr< monty::ndarray< int,1 > > _5499,std::shared_ptr< monty::ndarray< int,1 > > _5500,std::shared_ptr< monty::ndarray< double,1 > > _5501,monty::rc_ptr< ::mosek::fusion::Expression > _5502,int _5503);
void _initialize(int _5497,int _5498,std::shared_ptr< monty::ndarray< int,1 > > _5499,std::shared_ptr< monty::ndarray< int,1 > > _5500,std::shared_ptr< monty::ndarray< double,1 > > _5501,monty::rc_ptr< ::mosek::fusion::Expression > _5502,int _5503);
static ExprMulRight::t _new_ExprMulRight(int _5504,int _5505,std::shared_ptr< monty::ndarray< int,1 > > _5506,std::shared_ptr< monty::ndarray< int,1 > > _5507,std::shared_ptr< monty::ndarray< double,1 > > _5508,monty::rc_ptr< ::mosek::fusion::Expression > _5509);
void _initialize(int _5504,int _5505,std::shared_ptr< monty::ndarray< int,1 > > _5506,std::shared_ptr< monty::ndarray< int,1 > > _5507,std::shared_ptr< monty::ndarray< double,1 > > _5508,monty::rc_ptr< ::mosek::fusion::Expression > _5509);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5510,monty::rc_ptr< ::mosek::fusion::WorkStack > _5511,monty::rc_ptr< ::mosek::fusion::WorkStack > _5512) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(int _5656,std::shared_ptr< monty::ndarray< int,1 > > _5657);
static  int validate(int _5658,int _5659,std::shared_ptr< monty::ndarray< int,1 > > _5660,std::shared_ptr< monty::ndarray< int,1 > > _5661,std::shared_ptr< monty::ndarray< double,1 > > _5662,monty::rc_ptr< ::mosek::fusion::Expression > _5663);
virtual /* override */ std::string toString() ;
}; // struct ExprMulRight;

struct p_ExprMulLeft : public ::mosek::fusion::p_BaseExpression
{
ExprMulLeft * _pubthis;
static mosek::fusion::p_ExprMulLeft* _get_impl(mosek::fusion::ExprMulLeft * _inst){ return static_cast< mosek::fusion::p_ExprMulLeft* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulLeft * _get_impl(mosek::fusion::ExprMulLeft::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulLeft(ExprMulLeft * _pubthis);
virtual ~p_ExprMulLeft() { /* std::cout << "~p_ExprMulLeft" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};std::shared_ptr< monty::ndarray< double,1 > > mval{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdim1{};int mdim0{};virtual void destroy();
static ExprMulLeft::t _new_ExprMulLeft(int _5672,int _5673,std::shared_ptr< monty::ndarray< int,1 > > _5674,std::shared_ptr< monty::ndarray< int,1 > > _5675,std::shared_ptr< monty::ndarray< double,1 > > _5676,monty::rc_ptr< ::mosek::fusion::Expression > _5677,int _5678);
void _initialize(int _5672,int _5673,std::shared_ptr< monty::ndarray< int,1 > > _5674,std::shared_ptr< monty::ndarray< int,1 > > _5675,std::shared_ptr< monty::ndarray< double,1 > > _5676,monty::rc_ptr< ::mosek::fusion::Expression > _5677,int _5678);
static ExprMulLeft::t _new_ExprMulLeft(int _5679,int _5680,std::shared_ptr< monty::ndarray< int,1 > > _5681,std::shared_ptr< monty::ndarray< int,1 > > _5682,std::shared_ptr< monty::ndarray< double,1 > > _5683,monty::rc_ptr< ::mosek::fusion::Expression > _5684);
void _initialize(int _5679,int _5680,std::shared_ptr< monty::ndarray< int,1 > > _5681,std::shared_ptr< monty::ndarray< int,1 > > _5682,std::shared_ptr< monty::ndarray< double,1 > > _5683,monty::rc_ptr< ::mosek::fusion::Expression > _5684);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5685,monty::rc_ptr< ::mosek::fusion::WorkStack > _5686,monty::rc_ptr< ::mosek::fusion::WorkStack > _5687) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeshape(int _5789,int _5790,std::shared_ptr< monty::ndarray< int,1 > > _5791);
static  int validate(int _5792,int _5793,std::shared_ptr< monty::ndarray< int,1 > > _5794,std::shared_ptr< monty::ndarray< int,1 > > _5795,std::shared_ptr< monty::ndarray< double,1 > > _5796,monty::rc_ptr< ::mosek::fusion::Expression > _5797);
virtual /* override */ std::string toString() ;
}; // struct ExprMulLeft;

struct p_ExprMulVar : public ::mosek::fusion::p_BaseExpression
{
ExprMulVar * _pubthis;
static mosek::fusion::p_ExprMulVar* _get_impl(mosek::fusion::ExprMulVar * _inst){ return static_cast< mosek::fusion::p_ExprMulVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulVar * _get_impl(mosek::fusion::ExprMulVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulVar(ExprMulVar * _pubthis);
virtual ~p_ExprMulVar() { /* std::cout << "~p_ExprMulVar" << std::endl;*/ };
bool left{};monty::rc_ptr< ::mosek::fusion::Variable > x{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdimj{};int mdimi{};virtual void destroy();
static ExprMulVar::t _new_ExprMulVar(bool _5805,int _5806,int _5807,std::shared_ptr< monty::ndarray< int,1 > > _5808,std::shared_ptr< monty::ndarray< int,1 > > _5809,std::shared_ptr< monty::ndarray< double,1 > > _5810,monty::rc_ptr< ::mosek::fusion::Variable > _5811);
void _initialize(bool _5805,int _5806,int _5807,std::shared_ptr< monty::ndarray< int,1 > > _5808,std::shared_ptr< monty::ndarray< int,1 > > _5809,std::shared_ptr< monty::ndarray< double,1 > > _5810,monty::rc_ptr< ::mosek::fusion::Variable > _5811);
static ExprMulVar::t _new_ExprMulVar(bool _5814,int _5815,int _5816,std::shared_ptr< monty::ndarray< int,1 > > _5817,std::shared_ptr< monty::ndarray< int,1 > > _5818,std::shared_ptr< monty::ndarray< double,1 > > _5819,monty::rc_ptr< ::mosek::fusion::Variable > _5820,int _5821);
void _initialize(bool _5814,int _5815,int _5816,std::shared_ptr< monty::ndarray< int,1 > > _5817,std::shared_ptr< monty::ndarray< int,1 > > _5818,std::shared_ptr< monty::ndarray< double,1 > > _5819,monty::rc_ptr< ::mosek::fusion::Variable > _5820,int _5821);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _5822,monty::rc_ptr< ::mosek::fusion::WorkStack > _5823,monty::rc_ptr< ::mosek::fusion::WorkStack > _5824) ;
virtual void eval_right(monty::rc_ptr< ::mosek::fusion::WorkStack > _5825,monty::rc_ptr< ::mosek::fusion::WorkStack > _5826,monty::rc_ptr< ::mosek::fusion::WorkStack > _5827) ;
virtual void eval_left(monty::rc_ptr< ::mosek::fusion::WorkStack > _5932,monty::rc_ptr< ::mosek::fusion::WorkStack > _5933,monty::rc_ptr< ::mosek::fusion::WorkStack > _5934) ;
virtual void validate(int _6007,int _6008,std::shared_ptr< monty::ndarray< int,1 > > _6009,std::shared_ptr< monty::ndarray< int,1 > > _6010,std::shared_ptr< monty::ndarray< double,1 > > _6011) ;
static  std::shared_ptr< monty::ndarray< int,1 > > resshape(int _6015,int _6016,std::shared_ptr< monty::ndarray< int,1 > > _6017,bool _6018);
virtual /* override */ std::string toString() ;
}; // struct ExprMulVar;

struct p_ExprMulScalarVar : public ::mosek::fusion::p_BaseExpression
{
ExprMulScalarVar * _pubthis;
static mosek::fusion::p_ExprMulScalarVar* _get_impl(mosek::fusion::ExprMulScalarVar * _inst){ return static_cast< mosek::fusion::p_ExprMulScalarVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulScalarVar * _get_impl(mosek::fusion::ExprMulScalarVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulScalarVar(ExprMulScalarVar * _pubthis);
virtual ~p_ExprMulScalarVar() { /* std::cout << "~p_ExprMulScalarVar" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Variable > x{};std::shared_ptr< monty::ndarray< double,1 > > mcof{};std::shared_ptr< monty::ndarray< int,1 > > msubj{};std::shared_ptr< monty::ndarray< int,1 > > msubi{};int mdimj{};int mdimi{};virtual void destroy();
static ExprMulScalarVar::t _new_ExprMulScalarVar(int _6019,int _6020,std::shared_ptr< monty::ndarray< int,1 > > _6021,std::shared_ptr< monty::ndarray< int,1 > > _6022,std::shared_ptr< monty::ndarray< double,1 > > _6023,monty::rc_ptr< ::mosek::fusion::Variable > _6024);
void _initialize(int _6019,int _6020,std::shared_ptr< monty::ndarray< int,1 > > _6021,std::shared_ptr< monty::ndarray< int,1 > > _6022,std::shared_ptr< monty::ndarray< double,1 > > _6023,monty::rc_ptr< ::mosek::fusion::Variable > _6024);
static ExprMulScalarVar::t _new_ExprMulScalarVar(int _6029,int _6030,std::shared_ptr< monty::ndarray< int,1 > > _6031,std::shared_ptr< monty::ndarray< int,1 > > _6032,std::shared_ptr< monty::ndarray< double,1 > > _6033,monty::rc_ptr< ::mosek::fusion::Variable > _6034,int _6035);
void _initialize(int _6029,int _6030,std::shared_ptr< monty::ndarray< int,1 > > _6031,std::shared_ptr< monty::ndarray< int,1 > > _6032,std::shared_ptr< monty::ndarray< double,1 > > _6033,monty::rc_ptr< ::mosek::fusion::Variable > _6034,int _6035);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6036,monty::rc_ptr< ::mosek::fusion::WorkStack > _6037,monty::rc_ptr< ::mosek::fusion::WorkStack > _6038) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulScalarVar;

struct p_ExprMulVarScalarConst : public ::mosek::fusion::p_BaseExpression
{
ExprMulVarScalarConst * _pubthis;
static mosek::fusion::p_ExprMulVarScalarConst* _get_impl(mosek::fusion::ExprMulVarScalarConst * _inst){ return static_cast< mosek::fusion::p_ExprMulVarScalarConst* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprMulVarScalarConst * _get_impl(mosek::fusion::ExprMulVarScalarConst::t _inst) { return _get_impl(_inst.get()); }
p_ExprMulVarScalarConst(ExprMulVarScalarConst * _pubthis);
virtual ~p_ExprMulVarScalarConst() { /* std::cout << "~p_ExprMulVarScalarConst" << std::endl;*/ };
double c{};monty::rc_ptr< ::mosek::fusion::Variable > x{};virtual void destroy();
static ExprMulVarScalarConst::t _new_ExprMulVarScalarConst(monty::rc_ptr< ::mosek::fusion::Variable > _6055,double _6056);
void _initialize(monty::rc_ptr< ::mosek::fusion::Variable > _6055,double _6056);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6057,monty::rc_ptr< ::mosek::fusion::WorkStack > _6058,monty::rc_ptr< ::mosek::fusion::WorkStack > _6059) ;
virtual /* override */ std::string toString() ;
}; // struct ExprMulVarScalarConst;

struct p_ExprAdd : public ::mosek::fusion::p_BaseExpression
{
ExprAdd * _pubthis;
static mosek::fusion::p_ExprAdd* _get_impl(mosek::fusion::ExprAdd * _inst){ return static_cast< mosek::fusion::p_ExprAdd* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprAdd * _get_impl(mosek::fusion::ExprAdd::t _inst) { return _get_impl(_inst.get()); }
p_ExprAdd(ExprAdd * _pubthis);
virtual ~p_ExprAdd() { /* std::cout << "~p_ExprAdd" << std::endl;*/ };
double m2{};double m1{};monty::rc_ptr< ::mosek::fusion::Expression > e2{};monty::rc_ptr< ::mosek::fusion::Expression > e1{};virtual void destroy();
static ExprAdd::t _new_ExprAdd(monty::rc_ptr< ::mosek::fusion::Expression > _6076,monty::rc_ptr< ::mosek::fusion::Expression > _6077,double _6078,double _6079);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _6076,monty::rc_ptr< ::mosek::fusion::Expression > _6077,double _6078,double _6079);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6081,monty::rc_ptr< ::mosek::fusion::WorkStack > _6082,monty::rc_ptr< ::mosek::fusion::WorkStack > _6083) ;
virtual /* override */ std::string toString() ;
}; // struct ExprAdd;

struct p_ExprWSum : public ::mosek::fusion::p_BaseExpression
{
ExprWSum * _pubthis;
static mosek::fusion::p_ExprWSum* _get_impl(mosek::fusion::ExprWSum * _inst){ return static_cast< mosek::fusion::p_ExprWSum* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprWSum * _get_impl(mosek::fusion::ExprWSum::t _inst) { return _get_impl(_inst.get()); }
p_ExprWSum(ExprWSum * _pubthis);
virtual ~p_ExprWSum() { /* std::cout << "~p_ExprWSum" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > w{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > es{};virtual void destroy();
static ExprWSum::t _new_ExprWSum(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _6217,std::shared_ptr< monty::ndarray< double,1 > > _6218);
void _initialize(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _6217,std::shared_ptr< monty::ndarray< double,1 > > _6218);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6225,monty::rc_ptr< ::mosek::fusion::WorkStack > _6226,monty::rc_ptr< ::mosek::fusion::WorkStack > _6227) ;
virtual /* override */ std::string toString() ;
}; // struct ExprWSum;

struct p_ExprSumReduce : public ::mosek::fusion::p_BaseExpression
{
ExprSumReduce * _pubthis;
static mosek::fusion::p_ExprSumReduce* _get_impl(mosek::fusion::ExprSumReduce * _inst){ return static_cast< mosek::fusion::p_ExprSumReduce* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSumReduce * _get_impl(mosek::fusion::ExprSumReduce::t _inst) { return _get_impl(_inst.get()); }
p_ExprSumReduce(ExprSumReduce * _pubthis);
virtual ~p_ExprSumReduce() { /* std::cout << "~p_ExprSumReduce" << std::endl;*/ };
int dim{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSumReduce::t _new_ExprSumReduce(int _6321,monty::rc_ptr< ::mosek::fusion::Expression > _6322);
void _initialize(int _6321,monty::rc_ptr< ::mosek::fusion::Expression > _6322);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6324,monty::rc_ptr< ::mosek::fusion::WorkStack > _6325,monty::rc_ptr< ::mosek::fusion::WorkStack > _6326) ;
static  std::shared_ptr< monty::ndarray< int,1 > > computeShape(int _6442,std::shared_ptr< monty::ndarray< int,1 > > _6443);
virtual /* override */ std::string toString() ;
}; // struct ExprSumReduce;

struct p_ExprDenseTril : public ::mosek::fusion::p_BaseExpression
{
ExprDenseTril * _pubthis;
static mosek::fusion::p_ExprDenseTril* _get_impl(mosek::fusion::ExprDenseTril * _inst){ return static_cast< mosek::fusion::p_ExprDenseTril* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprDenseTril * _get_impl(mosek::fusion::ExprDenseTril::t _inst) { return _get_impl(_inst.get()); }
p_ExprDenseTril(ExprDenseTril * _pubthis);
virtual ~p_ExprDenseTril() { /* std::cout << "~p_ExprDenseTril" << std::endl;*/ };
int dim1{};int dim0{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprDenseTril::t _new_ExprDenseTril(int _6447,int _6448,monty::rc_ptr< ::mosek::fusion::Expression > _6449,int _6450);
void _initialize(int _6447,int _6448,monty::rc_ptr< ::mosek::fusion::Expression > _6449,int _6450);
static ExprDenseTril::t _new_ExprDenseTril(int _6451,int _6452,monty::rc_ptr< ::mosek::fusion::Expression > _6453);
void _initialize(int _6451,int _6452,monty::rc_ptr< ::mosek::fusion::Expression > _6453);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6455,monty::rc_ptr< ::mosek::fusion::WorkStack > _6456,monty::rc_ptr< ::mosek::fusion::WorkStack > _6457) ;
virtual /* override */ std::string toString() ;
}; // struct ExprDenseTril;

struct p_ExprDense : public ::mosek::fusion::p_BaseExpression
{
ExprDense * _pubthis;
static mosek::fusion::p_ExprDense* _get_impl(mosek::fusion::ExprDense * _inst){ return static_cast< mosek::fusion::p_ExprDense* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprDense * _get_impl(mosek::fusion::ExprDense::t _inst) { return _get_impl(_inst.get()); }
p_ExprDense(ExprDense * _pubthis);
virtual ~p_ExprDense() { /* std::cout << "~p_ExprDense" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprDense::t _new_ExprDense(monty::rc_ptr< ::mosek::fusion::Expression > _6541);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _6541);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6542,monty::rc_ptr< ::mosek::fusion::WorkStack > _6543,monty::rc_ptr< ::mosek::fusion::WorkStack > _6544) ;
virtual /* override */ std::string toString() ;
}; // struct ExprDense;

struct p_ExprSymmetrize : public ::mosek::fusion::p_BaseExpression
{
ExprSymmetrize * _pubthis;
static mosek::fusion::p_ExprSymmetrize* _get_impl(mosek::fusion::ExprSymmetrize * _inst){ return static_cast< mosek::fusion::p_ExprSymmetrize* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprSymmetrize * _get_impl(mosek::fusion::ExprSymmetrize::t _inst) { return _get_impl(_inst.get()); }
p_ExprSymmetrize(ExprSymmetrize * _pubthis);
virtual ~p_ExprSymmetrize() { /* std::cout << "~p_ExprSymmetrize" << std::endl;*/ };
int dim1{};int dim0{};monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprSymmetrize::t _new_ExprSymmetrize(int _6585,int _6586,monty::rc_ptr< ::mosek::fusion::Expression > _6587,int _6588);
void _initialize(int _6585,int _6586,monty::rc_ptr< ::mosek::fusion::Expression > _6587,int _6588);
static ExprSymmetrize::t _new_ExprSymmetrize(int _6589,int _6590,monty::rc_ptr< ::mosek::fusion::Expression > _6591);
void _initialize(int _6589,int _6590,monty::rc_ptr< ::mosek::fusion::Expression > _6591);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6593,monty::rc_ptr< ::mosek::fusion::WorkStack > _6594,monty::rc_ptr< ::mosek::fusion::WorkStack > _6595) ;
virtual /* override */ std::string toString() ;
}; // struct ExprSymmetrize;

struct p_ExprCondense : public ::mosek::fusion::p_BaseExpression
{
ExprCondense * _pubthis;
static mosek::fusion::p_ExprCondense* _get_impl(mosek::fusion::ExprCondense * _inst){ return static_cast< mosek::fusion::p_ExprCondense* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprCondense * _get_impl(mosek::fusion::ExprCondense::t _inst) { return _get_impl(_inst.get()); }
p_ExprCondense(ExprCondense * _pubthis);
virtual ~p_ExprCondense() { /* std::cout << "~p_ExprCondense" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > expr{};virtual void destroy();
static ExprCondense::t _new_ExprCondense(monty::rc_ptr< ::mosek::fusion::Expression > _6719);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _6719);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6720,monty::rc_ptr< ::mosek::fusion::WorkStack > _6721,monty::rc_ptr< ::mosek::fusion::WorkStack > _6722) ;
virtual /* override */ std::string toString() ;
}; // struct ExprCondense;

struct p_ExprFromVar : public ::mosek::fusion::p_BaseExpression
{
ExprFromVar * _pubthis;
static mosek::fusion::p_ExprFromVar* _get_impl(mosek::fusion::ExprFromVar * _inst){ return static_cast< mosek::fusion::p_ExprFromVar* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprFromVar * _get_impl(mosek::fusion::ExprFromVar::t _inst) { return _get_impl(_inst.get()); }
p_ExprFromVar(ExprFromVar * _pubthis);
virtual ~p_ExprFromVar() { /* std::cout << "~p_ExprFromVar" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Variable > x{};virtual void destroy();
static ExprFromVar::t _new_ExprFromVar(monty::rc_ptr< ::mosek::fusion::Variable > _6726);
void _initialize(monty::rc_ptr< ::mosek::fusion::Variable > _6726);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6727,monty::rc_ptr< ::mosek::fusion::WorkStack > _6728,monty::rc_ptr< ::mosek::fusion::WorkStack > _6729) ;
virtual /* override */ std::string toString() ;
}; // struct ExprFromVar;

struct p_ExprReshape : public ::mosek::fusion::p_BaseExpression
{
ExprReshape * _pubthis;
static mosek::fusion::p_ExprReshape* _get_impl(mosek::fusion::ExprReshape * _inst){ return static_cast< mosek::fusion::p_ExprReshape* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_ExprReshape * _get_impl(mosek::fusion::ExprReshape::t _inst) { return _get_impl(_inst.get()); }
p_ExprReshape(ExprReshape * _pubthis);
virtual ~p_ExprReshape() { /* std::cout << "~p_ExprReshape" << std::endl;*/ };
monty::rc_ptr< ::mosek::fusion::Expression > e{};virtual void destroy();
static ExprReshape::t _new_ExprReshape(std::shared_ptr< monty::ndarray< int,1 > > _6746,monty::rc_ptr< ::mosek::fusion::Expression > _6747);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _6746,monty::rc_ptr< ::mosek::fusion::Expression > _6747);
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _6749,monty::rc_ptr< ::mosek::fusion::WorkStack > _6750,monty::rc_ptr< ::mosek::fusion::WorkStack > _6751) ;
virtual /* override */ std::string toString() ;
}; // struct ExprReshape;

struct p_Expr : public ::mosek::fusion::p_BaseExpression
{
Expr * _pubthis;
static mosek::fusion::p_Expr* _get_impl(mosek::fusion::Expr * _inst){ return static_cast< mosek::fusion::p_Expr* >(mosek::fusion::p_BaseExpression::_get_impl(_inst)); }
static mosek::fusion::p_Expr * _get_impl(mosek::fusion::Expr::t _inst) { return _get_impl(_inst.get()); }
p_Expr(Expr * _pubthis);
virtual ~p_Expr() { /* std::cout << "~p_Expr" << std::endl;*/ };
std::shared_ptr< monty::ndarray< long long,1 > > inst{};std::shared_ptr< monty::ndarray< double,1 > > cof_v{};std::shared_ptr< monty::ndarray< long long,1 > > subj{};std::shared_ptr< monty::ndarray< long long,1 > > ptrb{};std::shared_ptr< monty::ndarray< double,1 > > bfix{};std::shared_ptr< monty::ndarray< int,1 > > shape{};virtual void destroy();
static Expr::t _new_Expr(std::shared_ptr< monty::ndarray< long long,1 > > _6893,std::shared_ptr< monty::ndarray< long long,1 > > _6894,std::shared_ptr< monty::ndarray< double,1 > > _6895,std::shared_ptr< monty::ndarray< double,1 > > _6896,std::shared_ptr< monty::ndarray< int,1 > > _6897,std::shared_ptr< monty::ndarray< long long,1 > > _6898);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _6893,std::shared_ptr< monty::ndarray< long long,1 > > _6894,std::shared_ptr< monty::ndarray< double,1 > > _6895,std::shared_ptr< monty::ndarray< double,1 > > _6896,std::shared_ptr< monty::ndarray< int,1 > > _6897,std::shared_ptr< monty::ndarray< long long,1 > > _6898);
static Expr::t _new_Expr(std::shared_ptr< monty::ndarray< long long,1 > > _6909,std::shared_ptr< monty::ndarray< long long,1 > > _6910,std::shared_ptr< monty::ndarray< double,1 > > _6911,std::shared_ptr< monty::ndarray< double,1 > > _6912,std::shared_ptr< monty::ndarray< int,1 > > _6913,std::shared_ptr< monty::ndarray< long long,1 > > _6914,int _6915);
void _initialize(std::shared_ptr< monty::ndarray< long long,1 > > _6909,std::shared_ptr< monty::ndarray< long long,1 > > _6910,std::shared_ptr< monty::ndarray< double,1 > > _6911,std::shared_ptr< monty::ndarray< double,1 > > _6912,std::shared_ptr< monty::ndarray< int,1 > > _6913,std::shared_ptr< monty::ndarray< long long,1 > > _6914,int _6915);
static Expr::t _new_Expr(monty::rc_ptr< ::mosek::fusion::Expression > _6916);
void _initialize(monty::rc_ptr< ::mosek::fusion::Expression > _6916);
virtual long long prod(std::shared_ptr< monty::ndarray< int,1 > > _6941) ;
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > varstack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > >,1 > > _6944);
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > varstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _6947,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _6948);
static  monty::rc_ptr< ::mosek::fusion::Expression > condense(monty::rc_ptr< ::mosek::fusion::Expression > _6952);
static  monty::rc_ptr< ::mosek::fusion::Expression > flatten(monty::rc_ptr< ::mosek::fusion::Expression > _6953);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _6955,int _6956,int _6957);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _6958,int _6959);
static  monty::rc_ptr< ::mosek::fusion::Expression > reshape(monty::rc_ptr< ::mosek::fusion::Expression > _6960,std::shared_ptr< monty::ndarray< int,1 > > _6961);
virtual long long size() ;
static  monty::rc_ptr< ::mosek::fusion::Expression > zeros(std::shared_ptr< monty::ndarray< int,1 > > _6962);
static  monty::rc_ptr< ::mosek::fusion::Expression > zeros(int _6963);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones();
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(std::shared_ptr< monty::ndarray< int,1 > > _6964,std::shared_ptr< monty::ndarray< int,2 > > _6965);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(std::shared_ptr< monty::ndarray< int,1 > > _6966);
static  monty::rc_ptr< ::mosek::fusion::Expression > ones(int _6967);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _6968);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(monty::rc_ptr< ::mosek::fusion::Matrix > _6969);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(double _6978);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _6979,std::shared_ptr< monty::ndarray< int,2 > > _6980,double _6981);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _6989,std::shared_ptr< monty::ndarray< int,2 > > _6990,std::shared_ptr< monty::ndarray< double,1 > > _6991);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< int,1 > > _6999,double _7000);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(int _7001,double _7002);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,2 > > _7004);
static  monty::rc_ptr< ::mosek::fusion::Expression > constTerm(std::shared_ptr< monty::ndarray< double,1 > > _7007);
virtual long long numNonzeros() ;
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > _7008,int _7009);
static  monty::rc_ptr< ::mosek::fusion::Expression > sum(monty::rc_ptr< ::mosek::fusion::Expression > _7010);
static  monty::rc_ptr< ::mosek::fusion::Expression > neg(monty::rc_ptr< ::mosek::fusion::Expression > _7011);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(bool _7012,monty::rc_ptr< ::mosek::fusion::Matrix > _7013,monty::rc_ptr< ::mosek::fusion::Expression > _7014);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > _7021,monty::rc_ptr< ::mosek::fusion::Parameter > _7022);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Parameter > _7023,monty::rc_ptr< ::mosek::fusion::Variable > _7024);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > _7025,monty::rc_ptr< ::mosek::fusion::Parameter > _7026);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Parameter > _7027,monty::rc_ptr< ::mosek::fusion::Expression > _7028);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > _7029,monty::rc_ptr< ::mosek::fusion::Matrix > _7030);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > _7031,monty::rc_ptr< ::mosek::fusion::Variable > _7032);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > _7033,monty::rc_ptr< ::mosek::fusion::Matrix > _7034);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Matrix > _7035,monty::rc_ptr< ::mosek::fusion::Expression > _7036);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Variable > _7037,std::shared_ptr< monty::ndarray< double,2 > > _7038);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(monty::rc_ptr< ::mosek::fusion::Expression > _7045,std::shared_ptr< monty::ndarray< double,2 > > _7046);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > _7053,monty::rc_ptr< ::mosek::fusion::Variable > _7054);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulDiag(std::shared_ptr< monty::ndarray< double,2 > > _7061,monty::rc_ptr< ::mosek::fusion::Expression > _7062);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(monty::rc_ptr< ::mosek::fusion::Matrix > _7069,monty::rc_ptr< ::mosek::fusion::Expression > _7070);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(std::shared_ptr< monty::ndarray< double,1 > > _7079,monty::rc_ptr< ::mosek::fusion::Expression > _7080);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm_(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7082,monty::rc_ptr< ::mosek::fusion::Expression > _7083);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _7086,double _7087);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(double _7088,monty::rc_ptr< ::mosek::fusion::Expression > _7089);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _7090,std::shared_ptr< monty::ndarray< double,1 > > _7091);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,1 > > _7092,monty::rc_ptr< ::mosek::fusion::Expression > _7093);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _7094,std::shared_ptr< monty::ndarray< double,2 > > _7095);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(std::shared_ptr< monty::ndarray< double,2 > > _7096,monty::rc_ptr< ::mosek::fusion::Expression > _7097);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _7098,monty::rc_ptr< ::mosek::fusion::Matrix > _7099);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > _7100,monty::rc_ptr< ::mosek::fusion::Expression > _7101);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _7102,std::shared_ptr< monty::ndarray< double,1 > > _7103,monty::rc_ptr< ::mosek::fusion::Expression > _7104);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _7119,std::shared_ptr< monty::ndarray< double,2 > > _7120,monty::rc_ptr< ::mosek::fusion::Expression > _7121);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _7136,monty::rc_ptr< ::mosek::fusion::Matrix > _7137,monty::rc_ptr< ::mosek::fusion::Expression > _7138);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Variable > _7147,monty::rc_ptr< ::mosek::fusion::Matrix > _7148);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Matrix > _7154,monty::rc_ptr< ::mosek::fusion::Variable > _7155);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(bool _7161,int _7162,int _7163,std::shared_ptr< monty::ndarray< int,1 > > _7164,std::shared_ptr< monty::ndarray< int,1 > > _7165,std::shared_ptr< monty::ndarray< double,1 > > _7166,monty::rc_ptr< ::mosek::fusion::Variable > _7167);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Expression > _7169,monty::rc_ptr< ::mosek::fusion::Parameter > _7170);
static  monty::rc_ptr< ::mosek::fusion::Expression > mul(monty::rc_ptr< ::mosek::fusion::Parameter > _7171,monty::rc_ptr< ::mosek::fusion::Expression > _7172);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _7173,monty::rc_ptr< ::mosek::fusion::Matrix > _7174);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _7182,std::shared_ptr< monty::ndarray< double,2 > > _7183);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _7187,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7188);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _7189,std::shared_ptr< monty::ndarray< double,1 > > _7190);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Matrix > _7195,monty::rc_ptr< ::mosek::fusion::Expression > _7196);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7197,monty::rc_ptr< ::mosek::fusion::Expression > _7198);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,2 > > _7199,monty::rc_ptr< ::mosek::fusion::Expression > _7200);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(std::shared_ptr< monty::ndarray< double,1 > > _7201,monty::rc_ptr< ::mosek::fusion::Expression > _7202);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Expression > _7203,monty::rc_ptr< ::mosek::fusion::Parameter > _7204);
static  monty::rc_ptr< ::mosek::fusion::Expression > dot(monty::rc_ptr< ::mosek::fusion::Parameter > _7205,monty::rc_ptr< ::mosek::fusion::Expression > _7206);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Parameter > _7207,monty::rc_ptr< ::mosek::fusion::Expression > _7208);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Expression > _7211,monty::rc_ptr< ::mosek::fusion::Parameter > _7212);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Matrix > _7215,monty::rc_ptr< ::mosek::fusion::Expression > _7216);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Expression > _7218,monty::rc_ptr< ::mosek::fusion::Matrix > _7219);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(std::shared_ptr< monty::ndarray< double,1 > > _7221,monty::rc_ptr< ::mosek::fusion::Expression > _7222);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer(monty::rc_ptr< ::mosek::fusion::Expression > _7224,std::shared_ptr< monty::ndarray< double,1 > > _7225);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer_(int _7227,std::shared_ptr< monty::ndarray< long long,1 > > _7228,std::shared_ptr< monty::ndarray< long long,1 > > _7229,std::shared_ptr< monty::ndarray< double,1 > > _7230,std::shared_ptr< monty::ndarray< double,1 > > _7231,std::shared_ptr< monty::ndarray< long long,1 > > _7232,std::shared_ptr< monty::ndarray< double,1 > > _7233,std::shared_ptr< monty::ndarray< int,1 > > _7234,int _7235,bool _7236);
static  monty::rc_ptr< ::mosek::fusion::Expression > outer_(monty::rc_ptr< ::mosek::fusion::Variable > _7266,int _7267,std::shared_ptr< monty::ndarray< double,1 > > _7268,std::shared_ptr< monty::ndarray< int,1 > > _7269,int _7270,bool _7271);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > >,1 > > _7288);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _7294,double _7295,double _7296);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _7297,double _7298,monty::rc_ptr< ::mosek::fusion::Expression > _7299);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _7300,monty::rc_ptr< ::mosek::fusion::Expression > _7301,double _7302);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _7303,monty::rc_ptr< ::mosek::fusion::Expression > _7304,monty::rc_ptr< ::mosek::fusion::Expression > _7305);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7306,double _7307,double _7308);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7309,double _7310,monty::rc_ptr< ::mosek::fusion::Expression > _7311);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7312,monty::rc_ptr< ::mosek::fusion::Expression > _7313,double _7314);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7315,monty::rc_ptr< ::mosek::fusion::Expression > _7316,monty::rc_ptr< ::mosek::fusion::Expression > _7317);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(double _7318,monty::rc_ptr< ::mosek::fusion::Expression > _7319);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7320,double _7321);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(monty::rc_ptr< ::mosek::fusion::Expression > _7322,monty::rc_ptr< ::mosek::fusion::Expression > _7323);
static  monty::rc_ptr< ::mosek::fusion::Expression > vstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7324);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7326,monty::rc_ptr< ::mosek::fusion::Expression > _7327,monty::rc_ptr< ::mosek::fusion::Expression > _7328);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7329,monty::rc_ptr< ::mosek::fusion::Expression > _7330,double _7331);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7332,double _7333,monty::rc_ptr< ::mosek::fusion::Expression > _7334);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7335,double _7336,double _7337);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _7338,monty::rc_ptr< ::mosek::fusion::Expression > _7339,monty::rc_ptr< ::mosek::fusion::Expression > _7340);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _7341,monty::rc_ptr< ::mosek::fusion::Expression > _7342,double _7343);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _7344,double _7345,monty::rc_ptr< ::mosek::fusion::Expression > _7346);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(double _7347,monty::rc_ptr< ::mosek::fusion::Expression > _7348);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7349,double _7350);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(monty::rc_ptr< ::mosek::fusion::Expression > _7351,monty::rc_ptr< ::mosek::fusion::Expression > _7352);
static  monty::rc_ptr< ::mosek::fusion::Expression > hstack(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7353);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7355,monty::rc_ptr< ::mosek::fusion::Expression > _7356,monty::rc_ptr< ::mosek::fusion::Expression > _7357,monty::rc_ptr< ::mosek::fusion::Expression > _7358);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7359,monty::rc_ptr< ::mosek::fusion::Expression > _7360,monty::rc_ptr< ::mosek::fusion::Expression > _7361,double _7362);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7363,monty::rc_ptr< ::mosek::fusion::Expression > _7364,double _7365,monty::rc_ptr< ::mosek::fusion::Expression > _7366);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7367,monty::rc_ptr< ::mosek::fusion::Expression > _7368,double _7369,double _7370);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7371,double _7372,monty::rc_ptr< ::mosek::fusion::Expression > _7373,monty::rc_ptr< ::mosek::fusion::Expression > _7374);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7375,double _7376,monty::rc_ptr< ::mosek::fusion::Expression > _7377,double _7378);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7379,double _7380,double _7381,monty::rc_ptr< ::mosek::fusion::Expression > _7382);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7383,double _7384,monty::rc_ptr< ::mosek::fusion::Expression > _7385);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7386,monty::rc_ptr< ::mosek::fusion::Expression > _7387,double _7388);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7389,monty::rc_ptr< ::mosek::fusion::Expression > _7390,monty::rc_ptr< ::mosek::fusion::Expression > _7391);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack(int _7392,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7393);
static  monty::rc_ptr< ::mosek::fusion::Expression > stack_(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7394,int _7395);
static  std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > promote(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7396,int _7397);
static  monty::rc_ptr< ::mosek::fusion::Expression > repeat(monty::rc_ptr< ::mosek::fusion::Variable > _7410,int _7411,int _7412);
static  monty::rc_ptr< ::mosek::fusion::Expression > repeat(monty::rc_ptr< ::mosek::fusion::Expression > _7413,int _7414,int _7415);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Expression >,1 > > _7418);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _7420);
static  monty::rc_ptr< ::mosek::fusion::Expression > add_(monty::rc_ptr< ::mosek::fusion::Expression > _7453,double _7454,monty::rc_ptr< ::mosek::fusion::Expression > _7455,double _7456);
static  monty::rc_ptr< ::mosek::fusion::Expression > transpose(monty::rc_ptr< ::mosek::fusion::Expression > _7467);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Matrix > _7468,monty::rc_ptr< ::mosek::fusion::Expression > _7469);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7470,monty::rc_ptr< ::mosek::fusion::Expression > _7471);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,2 > > _7472,monty::rc_ptr< ::mosek::fusion::Expression > _7473);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(std::shared_ptr< monty::ndarray< double,1 > > _7474,monty::rc_ptr< ::mosek::fusion::Expression > _7475);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _7476,monty::rc_ptr< ::mosek::fusion::Matrix > _7477);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _7478,std::shared_ptr< monty::ndarray< double,2 > > _7479);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _7480,std::shared_ptr< monty::ndarray< double,1 > > _7481);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _7482,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7483);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Parameter > _7484,monty::rc_ptr< ::mosek::fusion::Expression > _7485);
static  monty::rc_ptr< ::mosek::fusion::Expression > mulElm(monty::rc_ptr< ::mosek::fusion::Expression > _7486,monty::rc_ptr< ::mosek::fusion::Parameter > _7487);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7488,monty::rc_ptr< ::mosek::fusion::Expression > _7489);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7490,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7491);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Matrix > _7492,monty::rc_ptr< ::mosek::fusion::Expression > _7493);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7494,monty::rc_ptr< ::mosek::fusion::Matrix > _7495);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(double _7496,monty::rc_ptr< ::mosek::fusion::Expression > _7497);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7498,double _7499);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,2 > > _7500,monty::rc_ptr< ::mosek::fusion::Expression > _7501);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(std::shared_ptr< monty::ndarray< double,1 > > _7502,monty::rc_ptr< ::mosek::fusion::Expression > _7503);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7504,std::shared_ptr< monty::ndarray< double,2 > > _7505);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7506,std::shared_ptr< monty::ndarray< double,1 > > _7507);
static  monty::rc_ptr< ::mosek::fusion::Expression > sub(monty::rc_ptr< ::mosek::fusion::Expression > _7508,monty::rc_ptr< ::mosek::fusion::Expression > _7509);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7510,monty::rc_ptr< ::mosek::fusion::Expression > _7511);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7512,monty::rc_ptr< ::mosek::fusion::NDSparseArray > _7513);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Matrix > _7514,monty::rc_ptr< ::mosek::fusion::Expression > _7515);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7516,monty::rc_ptr< ::mosek::fusion::Matrix > _7517);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(double _7518,monty::rc_ptr< ::mosek::fusion::Expression > _7519);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7520,double _7521);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,2 > > _7522,monty::rc_ptr< ::mosek::fusion::Expression > _7523);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(std::shared_ptr< monty::ndarray< double,1 > > _7524,monty::rc_ptr< ::mosek::fusion::Expression > _7525);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7526,std::shared_ptr< monty::ndarray< double,2 > > _7527);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7528,std::shared_ptr< monty::ndarray< double,1 > > _7529);
static  monty::rc_ptr< ::mosek::fusion::Expression > add(monty::rc_ptr< ::mosek::fusion::Expression > _7530,monty::rc_ptr< ::mosek::fusion::Expression > _7531);
virtual /* override */ int getND() ;
virtual /* override */ std::shared_ptr< monty::ndarray< int,1 > > getShape() ;
virtual /* override */ void eval(monty::rc_ptr< ::mosek::fusion::WorkStack > _7532,monty::rc_ptr< ::mosek::fusion::WorkStack > _7533,monty::rc_ptr< ::mosek::fusion::WorkStack > _7534) ;
static  void validateData(std::shared_ptr< monty::ndarray< long long,1 > > _7550,std::shared_ptr< monty::ndarray< long long,1 > > _7551,std::shared_ptr< monty::ndarray< double,1 > > _7552,std::shared_ptr< monty::ndarray< double,1 > > _7553,std::shared_ptr< monty::ndarray< int,1 > > _7554,std::shared_ptr< monty::ndarray< long long,1 > > _7555);
static  monty::rc_ptr< ::mosek::fusion::Model > extractModel(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _7568);
}; // struct Expr;

struct p_WorkStack
{
WorkStack * _pubthis;
static mosek::fusion::p_WorkStack* _get_impl(mosek::fusion::WorkStack * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_WorkStack * _get_impl(mosek::fusion::WorkStack::t _inst) { return _get_impl(_inst.get()); }
p_WorkStack(WorkStack * _pubthis);
virtual ~p_WorkStack() { /* std::cout << "~p_WorkStack" << std::endl;*/ };
int code_base{};int cconst_base{};int codeptr_base{};int cof_base{};int nidxs_base{};int sp_base{};int shape_base{};int ptr_base{};bool hassp{};int ncodeatom{};int nelem{};int nnz{};int nd{};int pf64{};int pi64{};int pi32{};std::shared_ptr< monty::ndarray< double,1 > > f64{};std::shared_ptr< monty::ndarray< long long,1 > > i64{};std::shared_ptr< monty::ndarray< int,1 > > i32{};virtual void destroy();
static WorkStack::t _new_WorkStack();
void _initialize();
virtual std::string formatCurrent() ;
virtual bool peek_hassp() ;
virtual int peek_nnz() ;
virtual int peek_nelem() ;
virtual int peek_dim(int _6814) ;
virtual int peek_nd() ;
virtual void alloc_expr(int _6815,int _6816,int _6817,bool _6818) ;
virtual void alloc_expr(int _6819,int _6820,int _6821,bool _6822,int _6823) ;
virtual void pop_expr() ;
virtual void move_expr(monty::rc_ptr< ::mosek::fusion::WorkStack > _6824) ;
virtual void peek_expr() ;
virtual void ensure_sparsity() ;
virtual void clear() ;
virtual int allocf64(int _6839) ;
virtual int alloci64(int _6841) ;
virtual int alloci32(int _6843) ;
virtual void pushf64(double _6845) ;
virtual void pushi64(long long _6846) ;
virtual void pushi32(int _6847) ;
virtual void ensuref64(int _6848) ;
virtual void ensurei64(int _6851) ;
virtual void ensurei32(int _6854) ;
virtual int popf64(int _6857) ;
virtual int popi64(int _6858) ;
virtual int popi32(int _6859) ;
virtual void popf64(int _6860,std::shared_ptr< monty::ndarray< double,1 > > _6861,int _6862) ;
virtual void popi64(int _6863,std::shared_ptr< monty::ndarray< long long,1 > > _6864,int _6865) ;
virtual void popi32(int _6866,std::shared_ptr< monty::ndarray< int,1 > > _6867,int _6868) ;
virtual double popf64() ;
virtual long long popi64() ;
virtual int popi32() ;
virtual double peekf64() ;
virtual long long peeki64() ;
virtual int peeki32() ;
virtual double peekf64(int _6869) ;
virtual long long peeki64(int _6870) ;
virtual int peeki32(int _6871) ;
}; // struct WorkStack;

struct p_SymmetricExpr
{
SymmetricExpr * _pubthis;
static mosek::fusion::p_SymmetricExpr* _get_impl(mosek::fusion::SymmetricExpr * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricExpr * _get_impl(mosek::fusion::SymmetricExpr::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricExpr(SymmetricExpr * _pubthis);
virtual ~p_SymmetricExpr() { /* std::cout << "~p_SymmetricExpr" << std::endl;*/ };
std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > xs{};monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > b{};std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > Ms{};int n{};virtual void destroy();
static SymmetricExpr::t _new_SymmetricExpr(int _6872,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > _6873,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _6874,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _6875);
void _initialize(int _6872,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::SymmetricMatrix >,1 > > _6873,std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Variable >,1 > > _6874,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _6875);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > add(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _6876,monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _6877);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > mul(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _6878,double _6879);
static  monty::rc_ptr< ::mosek::fusion::SymmetricExpr > add(monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _6881,monty::rc_ptr< ::mosek::fusion::SymmetricExpr > _6882);
virtual /* override */ std::string toString() ;
}; // struct SymmetricExpr;

struct p_SymmetricMatrix
{
SymmetricMatrix * _pubthis;
static mosek::fusion::p_SymmetricMatrix* _get_impl(mosek::fusion::SymmetricMatrix * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_SymmetricMatrix * _get_impl(mosek::fusion::SymmetricMatrix::t _inst) { return _get_impl(_inst.get()); }
p_SymmetricMatrix(SymmetricMatrix * _pubthis);
virtual ~p_SymmetricMatrix() { /* std::cout << "~p_SymmetricMatrix" << std::endl;*/ };
int nnz{};double scale{};std::shared_ptr< monty::ndarray< double,1 > > vval{};std::shared_ptr< monty::ndarray< int,1 > > vsubj{};std::shared_ptr< monty::ndarray< int,1 > > vsubi{};std::shared_ptr< monty::ndarray< double,1 > > uval{};std::shared_ptr< monty::ndarray< int,1 > > usubj{};std::shared_ptr< monty::ndarray< int,1 > > usubi{};int d1{};int d0{};virtual void destroy();
static SymmetricMatrix::t _new_SymmetricMatrix(int _7581,int _7582,std::shared_ptr< monty::ndarray< int,1 > > _7583,std::shared_ptr< monty::ndarray< int,1 > > _7584,std::shared_ptr< monty::ndarray< double,1 > > _7585,std::shared_ptr< monty::ndarray< int,1 > > _7586,std::shared_ptr< monty::ndarray< int,1 > > _7587,std::shared_ptr< monty::ndarray< double,1 > > _7588,double _7589);
void _initialize(int _7581,int _7582,std::shared_ptr< monty::ndarray< int,1 > > _7583,std::shared_ptr< monty::ndarray< int,1 > > _7584,std::shared_ptr< monty::ndarray< double,1 > > _7585,std::shared_ptr< monty::ndarray< int,1 > > _7586,std::shared_ptr< monty::ndarray< int,1 > > _7587,std::shared_ptr< monty::ndarray< double,1 > > _7588,double _7589);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(int _7590,std::shared_ptr< monty::ndarray< int,1 > > _7591,std::shared_ptr< monty::ndarray< double,1 > > _7592);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > rankOne(std::shared_ptr< monty::ndarray< double,1 > > _7600);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > antiDiag(std::shared_ptr< monty::ndarray< double,1 > > _7608);
static  monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _7615);
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__add(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _7621) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__sub(monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > _7641) ;
virtual monty::rc_ptr< ::mosek::fusion::SymmetricMatrix > __mosek_2fusion_2SymmetricMatrix__mul(double _7642) ;
virtual int getdim() ;
}; // struct SymmetricMatrix;

struct p_NDSparseArray
{
NDSparseArray * _pubthis;
static mosek::fusion::p_NDSparseArray* _get_impl(mosek::fusion::NDSparseArray * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_NDSparseArray * _get_impl(mosek::fusion::NDSparseArray::t _inst) { return _get_impl(_inst.get()); }
p_NDSparseArray(NDSparseArray * _pubthis);
virtual ~p_NDSparseArray() { /* std::cout << "~p_NDSparseArray" << std::endl;*/ };
std::shared_ptr< monty::ndarray< double,1 > > cof{};std::shared_ptr< monty::ndarray< long long,1 > > inst{};std::shared_ptr< monty::ndarray< int,1 > > dims{};long long size{};virtual void destroy();
static NDSparseArray::t _new_NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > _7643,std::shared_ptr< monty::ndarray< int,2 > > _7644,std::shared_ptr< monty::ndarray< double,1 > > _7645);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _7643,std::shared_ptr< monty::ndarray< int,2 > > _7644,std::shared_ptr< monty::ndarray< double,1 > > _7645);
static NDSparseArray::t _new_NDSparseArray(std::shared_ptr< monty::ndarray< int,1 > > _7666,std::shared_ptr< monty::ndarray< long long,1 > > _7667,std::shared_ptr< monty::ndarray< double,1 > > _7668);
void _initialize(std::shared_ptr< monty::ndarray< int,1 > > _7666,std::shared_ptr< monty::ndarray< long long,1 > > _7667,std::shared_ptr< monty::ndarray< double,1 > > _7668);
static NDSparseArray::t _new_NDSparseArray(monty::rc_ptr< ::mosek::fusion::Matrix > _7684);
void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix > _7684);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(monty::rc_ptr< ::mosek::fusion::Matrix > _7692);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > _7693,std::shared_ptr< monty::ndarray< long long,1 > > _7694,std::shared_ptr< monty::ndarray< double,1 > > _7695);
static  monty::rc_ptr< ::mosek::fusion::NDSparseArray > make(std::shared_ptr< monty::ndarray< int,1 > > _7696,std::shared_ptr< monty::ndarray< int,2 > > _7697,std::shared_ptr< monty::ndarray< double,1 > > _7698);
}; // struct NDSparseArray;

struct p_Matrix
{
Matrix * _pubthis;
static mosek::fusion::p_Matrix* _get_impl(mosek::fusion::Matrix * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Matrix * _get_impl(mosek::fusion::Matrix::t _inst) { return _get_impl(_inst.get()); }
p_Matrix(Matrix * _pubthis);
virtual ~p_Matrix() { /* std::cout << "~p_Matrix" << std::endl;*/ };
int dimj{};int dimi{};virtual void destroy();
static Matrix::t _new_Matrix(int _7768,int _7769);
void _initialize(int _7768,int _7769);
virtual /* override */ std::string toString() ;
virtual void switchDims() ;
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _7771,monty::rc_ptr< ::mosek::fusion::Matrix > _7772);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > > _7774);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int _7792,double _7793,int _7794);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(int _7795,double _7796);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _7797,double _7798,int _7799);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(int _7800,double _7801);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > _7802,int _7803);
static  monty::rc_ptr< ::mosek::fusion::Matrix > antidiag(std::shared_ptr< monty::ndarray< double,1 > > _7813);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _7814,int _7815);
static  monty::rc_ptr< ::mosek::fusion::Matrix > diag(std::shared_ptr< monty::ndarray< double,1 > > _7823);
static  monty::rc_ptr< ::mosek::fusion::Matrix > ones(int _7824,int _7825);
static  monty::rc_ptr< ::mosek::fusion::Matrix > eye(int _7826);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(monty::rc_ptr< ::mosek::fusion::Matrix > _7828);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int _7829,int _7830,double _7831);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(int _7832,int _7833,std::shared_ptr< monty::ndarray< double,1 > > _7834);
static  monty::rc_ptr< ::mosek::fusion::Matrix > dense(std::shared_ptr< monty::ndarray< double,2 > > _7835);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(monty::rc_ptr< ::mosek::fusion::Matrix > _7836);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< std::shared_ptr< monty::ndarray< monty::rc_ptr< ::mosek::fusion::Matrix >,1 > >,1 > > _7840);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< double,2 > > _7871);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _7881,int _7882);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _7883,int _7884,std::shared_ptr< monty::ndarray< int,1 > > _7885,std::shared_ptr< monty::ndarray< int,1 > > _7886,double _7887);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > _7889,std::shared_ptr< monty::ndarray< int,1 > > _7890,double _7891);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(std::shared_ptr< monty::ndarray< int,1 > > _7896,std::shared_ptr< monty::ndarray< int,1 > > _7897,std::shared_ptr< monty::ndarray< double,1 > > _7898);
static  monty::rc_ptr< ::mosek::fusion::Matrix > sparse(int _7903,int _7904,std::shared_ptr< monty::ndarray< int,1 > > _7905,std::shared_ptr< monty::ndarray< int,1 > > _7906,std::shared_ptr< monty::ndarray< double,1 > > _7907);
virtual double get(int _7912,int _7913) { throw monty::AbstractClassError("Call to abstract method"); }
virtual bool isSparse() { throw monty::AbstractClassError("Call to abstract method"); }
virtual std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() { throw monty::AbstractClassError("Call to abstract method"); }
virtual void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _7914,std::shared_ptr< monty::ndarray< int,1 > > _7915,std::shared_ptr< monty::ndarray< double,1 > > _7916) { throw monty::AbstractClassError("Call to abstract method"); }
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { throw monty::AbstractClassError("Call to abstract method"); }
virtual long long numNonzeros() { throw monty::AbstractClassError("Call to abstract method"); }
virtual int numColumns() ;
virtual int numRows() ;
}; // struct Matrix;

struct p_DenseMatrix : public ::mosek::fusion::p_Matrix
{
DenseMatrix * _pubthis;
static mosek::fusion::p_DenseMatrix* _get_impl(mosek::fusion::DenseMatrix * _inst){ return static_cast< mosek::fusion::p_DenseMatrix* >(mosek::fusion::p_Matrix::_get_impl(_inst)); }
static mosek::fusion::p_DenseMatrix * _get_impl(mosek::fusion::DenseMatrix::t _inst) { return _get_impl(_inst.get()); }
p_DenseMatrix(DenseMatrix * _pubthis);
virtual ~p_DenseMatrix() { /* std::cout << "~p_DenseMatrix" << std::endl;*/ };
long long nnz{};std::shared_ptr< monty::ndarray< double,1 > > data{};virtual void destroy();
static DenseMatrix::t _new_DenseMatrix(int _7699,int _7700,std::shared_ptr< monty::ndarray< double,1 > > _7701);
void _initialize(int _7699,int _7700,std::shared_ptr< monty::ndarray< double,1 > > _7701);
static DenseMatrix::t _new_DenseMatrix(monty::rc_ptr< ::mosek::fusion::Matrix > _7702);
void _initialize(monty::rc_ptr< ::mosek::fusion::Matrix > _7702);
static DenseMatrix::t _new_DenseMatrix(std::shared_ptr< monty::ndarray< double,2 > > _7707);
void _initialize(std::shared_ptr< monty::ndarray< double,2 > > _7707);
static DenseMatrix::t _new_DenseMatrix(int _7710,int _7711,double _7712);
void _initialize(int _7710,int _7711,double _7712);
virtual /* override */ std::string toString() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2DenseMatrix__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { return __mosek_2fusion_2DenseMatrix__transpose(); }
virtual /* override */ bool isSparse() ;
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() ;
virtual /* override */ void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _7725,std::shared_ptr< monty::ndarray< int,1 > > _7726,std::shared_ptr< monty::ndarray< double,1 > > _7727) ;
virtual /* override */ double get(int _7731,int _7732) ;
virtual /* override */ long long numNonzeros() ;
}; // struct DenseMatrix;

struct p_SparseMatrix : public ::mosek::fusion::p_Matrix
{
SparseMatrix * _pubthis;
static mosek::fusion::p_SparseMatrix* _get_impl(mosek::fusion::SparseMatrix * _inst){ return static_cast< mosek::fusion::p_SparseMatrix* >(mosek::fusion::p_Matrix::_get_impl(_inst)); }
static mosek::fusion::p_SparseMatrix * _get_impl(mosek::fusion::SparseMatrix::t _inst) { return _get_impl(_inst.get()); }
p_SparseMatrix(SparseMatrix * _pubthis);
virtual ~p_SparseMatrix() { /* std::cout << "~p_SparseMatrix" << std::endl;*/ };
long long nnz{};std::shared_ptr< monty::ndarray< double,1 > > val{};std::shared_ptr< monty::ndarray< int,1 > > subj{};std::shared_ptr< monty::ndarray< int,1 > > subi{};virtual void destroy();
static SparseMatrix::t _new_SparseMatrix(int _7733,int _7734,std::shared_ptr< monty::ndarray< int,1 > > _7735,std::shared_ptr< monty::ndarray< int,1 > > _7736,std::shared_ptr< monty::ndarray< double,1 > > _7737,long long _7738);
void _initialize(int _7733,int _7734,std::shared_ptr< monty::ndarray< int,1 > > _7735,std::shared_ptr< monty::ndarray< int,1 > > _7736,std::shared_ptr< monty::ndarray< double,1 > > _7737,long long _7738);
static SparseMatrix::t _new_SparseMatrix(int _7744,int _7745,std::shared_ptr< monty::ndarray< int,1 > > _7746,std::shared_ptr< monty::ndarray< int,1 > > _7747,std::shared_ptr< monty::ndarray< double,1 > > _7748);
void _initialize(int _7744,int _7745,std::shared_ptr< monty::ndarray< int,1 > > _7746,std::shared_ptr< monty::ndarray< int,1 > > _7747,std::shared_ptr< monty::ndarray< double,1 > > _7748);
virtual std::shared_ptr< monty::ndarray< long long,1 > > formPtrb() ;
virtual /* override */ std::string toString() ;
virtual /* override */ long long numNonzeros() ;
virtual /* override */ monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2SparseMatrix__transpose() ;
virtual monty::rc_ptr< ::mosek::fusion::Matrix > __mosek_2fusion_2Matrix__transpose() { return __mosek_2fusion_2SparseMatrix__transpose(); }
virtual /* override */ bool isSparse() ;
virtual /* override */ std::shared_ptr< monty::ndarray< double,1 > > getDataAsArray() ;
virtual /* override */ void getDataAsTriplets(std::shared_ptr< monty::ndarray< int,1 > > _7760,std::shared_ptr< monty::ndarray< int,1 > > _7761,std::shared_ptr< monty::ndarray< double,1 > > _7762) ;
virtual /* override */ double get(int _7763,int _7764) ;
}; // struct SparseMatrix;

struct p_LinkedBlocks
{
LinkedBlocks * _pubthis;
static mosek::fusion::p_LinkedBlocks* _get_impl(mosek::fusion::LinkedBlocks * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinkedBlocks * _get_impl(mosek::fusion::LinkedBlocks::t _inst) { return _get_impl(_inst.get()); }
p_LinkedBlocks(LinkedBlocks * _pubthis);
virtual ~p_LinkedBlocks() { /* std::cout << "~p_LinkedBlocks" << std::endl;*/ };
std::shared_ptr< monty::ndarray< int,1 > > bfirst{};std::shared_ptr< monty::ndarray< int,1 > > bsize{};monty::rc_ptr< ::mosek::fusion::LinkedInts > blocks{};monty::rc_ptr< ::mosek::fusion::LinkedInts > ints{};virtual void destroy();
static LinkedBlocks::t _new_LinkedBlocks();
void _initialize();
static LinkedBlocks::t _new_LinkedBlocks(int _7942);
void _initialize(int _7942);
static LinkedBlocks::t _new_LinkedBlocks(monty::rc_ptr< ::mosek::fusion::LinkedBlocks > _7943);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinkedBlocks > _7943);
virtual void free(int _7944) ;
virtual int alloc(int _7946) ;
virtual int maxidx(int _7951) ;
virtual void get(int _7952,std::shared_ptr< monty::ndarray< int,1 > > _7953,int _7954) ;
virtual int numblocks() ;
virtual int blocksize(int _7955) ;
virtual int capacity() ;
virtual bool validate() ;
}; // struct LinkedBlocks;

struct p_LinkedInts
{
LinkedInts * _pubthis;
static mosek::fusion::p_LinkedInts* _get_impl(mosek::fusion::LinkedInts * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_LinkedInts * _get_impl(mosek::fusion::LinkedInts::t _inst) { return _get_impl(_inst.get()); }
p_LinkedInts(LinkedInts * _pubthis);
virtual ~p_LinkedInts() { /* std::cout << "~p_LinkedInts" << std::endl;*/ };
int nfree{};int last_free{};int first_free{};int first_used{};std::shared_ptr< monty::ndarray< int,1 > > prev{};std::shared_ptr< monty::ndarray< int,1 > > next{};virtual void destroy();
static LinkedInts::t _new_LinkedInts(int _7956);
void _initialize(int _7956);
static LinkedInts::t _new_LinkedInts();
void _initialize();
static LinkedInts::t _new_LinkedInts(monty::rc_ptr< ::mosek::fusion::LinkedInts > _7959);
void _initialize(monty::rc_ptr< ::mosek::fusion::LinkedInts > _7959);
virtual void free(int _7960,int _7961) ;
virtual int alloc() ;
virtual int alloc(int _7967) ;
virtual void alloc(int _7968,std::shared_ptr< monty::ndarray< int,1 > > _7969,int _7970) ;
virtual void get(int _7973,int _7974,std::shared_ptr< monty::ndarray< int,1 > > _7975,int _7976) ;
virtual int maxidx(int _7979,int _7980) ;
virtual int allocblock(int _7984) ;
virtual void recap(int _7990) ;
virtual int capacity() ;
virtual bool validate() ;
}; // struct LinkedInts;

struct p_Parameters
{
Parameters * _pubthis;
static mosek::fusion::p_Parameters* _get_impl(mosek::fusion::Parameters * _inst){ assert(_inst); assert(_inst->_impl); return _inst->_impl; }
static mosek::fusion::p_Parameters * _get_impl(mosek::fusion::Parameters::t _inst) { return _get_impl(_inst.get()); }
p_Parameters(Parameters * _pubthis);
virtual ~p_Parameters() { /* std::cout << "~p_Parameters" << std::endl;*/ };
virtual void destroy();
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _7999,const std::string &  _8000,double _8001);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _8095,const std::string &  _8096,int _8097);
static  void setParameter(monty::rc_ptr< ::mosek::fusion::Model > _8191,const std::string &  _8192,const std::string &  _8193);
static  int string_to_variabletype_value(const std::string &  _8431);
static  int string_to_value_value(const std::string &  _8432);
static  int string_to_streamtype_value(const std::string &  _8433);
static  int string_to_startpointtype_value(const std::string &  _8434);
static  int string_to_stakey_value(const std::string &  _8435);
static  int string_to_sparam_value(const std::string &  _8436);
static  int string_to_solveform_value(const std::string &  _8437);
static  int string_to_soltype_value(const std::string &  _8438);
static  int string_to_solsta_value(const std::string &  _8439);
static  int string_to_solitem_value(const std::string &  _8440);
static  int string_to_simseltype_value(const std::string &  _8441);
static  int string_to_sensitivitytype_value(const std::string &  _8442);
static  int string_to_scalingmethod_value(const std::string &  _8443);
static  int string_to_scalingtype_value(const std::string &  _8444);
static  int string_to_rescodetype_value(const std::string &  _8445);
static  int string_to_rescode_value(const std::string &  _8446);
static  int string_to_xmlwriteroutputtype_value(const std::string &  _8447);
static  int string_to_prosta_value(const std::string &  _8448);
static  int string_to_problemtype_value(const std::string &  _8449);
static  int string_to_problemitem_value(const std::string &  _8450);
static  int string_to_parametertype_value(const std::string &  _8451);
static  int string_to_presolvemode_value(const std::string &  _8452);
static  int string_to_orderingtype_value(const std::string &  _8453);
static  int string_to_optimizertype_value(const std::string &  _8454);
static  int string_to_onoffkey_value(const std::string &  _8455);
static  int string_to_objsense_value(const std::string &  _8456);
static  int string_to_mpsformat_value(const std::string &  _8457);
static  int string_to_mionodeseltype_value(const std::string &  _8458);
static  int string_to_miomode_value(const std::string &  _8459);
static  int string_to_miocontsoltype_value(const std::string &  _8460);
static  int string_to_branchdir_value(const std::string &  _8461);
static  int string_to_iparam_value(const std::string &  _8462);
static  int string_to_iomode_value(const std::string &  _8463);
static  int string_to_internal_iinf_value(const std::string &  _8464);
static  int string_to_internal_dinf_value(const std::string &  _8465);
static  int string_to_inftype_value(const std::string &  _8466);
static  int string_to_iinfitem_value(const std::string &  _8467);
static  int string_to_internal_liinf_value(const std::string &  _8468);
static  int string_to_liinfitem_value(const std::string &  _8469);
static  int string_to_dparam_value(const std::string &  _8470);
static  int string_to_feature_value(const std::string &  _8471);
static  int string_to_dinfitem_value(const std::string &  _8472);
static  int string_to_dataformat_value(const std::string &  _8473);
static  int string_to_symmattype_value(const std::string &  _8474);
static  int string_to_scopr_value(const std::string &  _8475);
static  int string_to_nametype_value(const std::string &  _8476);
static  int string_to_conetype_value(const std::string &  _8477);
static  int string_to_compresstype_value(const std::string &  _8478);
static  int string_to_checkconvexitytype_value(const std::string &  _8479);
static  int string_to_callbackcode_value(const std::string &  _8480);
static  int string_to_purify_value(const std::string &  _8481);
static  int string_to_intpnthotstart_value(const std::string &  _8482);
static  int string_to_simhotstart_value(const std::string &  _8483);
static  int string_to_simdupvec_value(const std::string &  _8484);
static  int string_to_simreform_value(const std::string &  _8485);
static  int string_to_uplo_value(const std::string &  _8486);
static  int string_to_transpose_value(const std::string &  _8487);
static  int string_to_simdegen_value(const std::string &  _8488);
static  int string_to_mark_value(const std::string &  _8489);
static  int string_to_boundkey_value(const std::string &  _8490);
static  int string_to_basindtype_value(const std::string &  _8491);
static  int string_to_language_value(const std::string &  _8492);
}; // struct Parameters;

}
}
namespace mosek
{
namespace fusion
{
namespace Utils
{
// mosek.fusion.Utils.IntMap from file 'src\fusion\cxx\IntMap_p.h'
struct p_IntMap 
{
  IntMap * _pubself;

  static p_IntMap * _get_impl(IntMap * _inst) { return _inst->_impl.get(); }

  p_IntMap(IntMap * _pubself) : _pubself(_pubself) {}

  static IntMap::t _new_IntMap() { return new IntMap(); }

  ::std::unordered_map<long long,int> m;

  bool hasItem (long long key) { return m.find(key) != m.end(); }
  int  getItem (long long key) { return m.find(key)->second; } // will probably throw something or crash of no such key
  void setItem (long long key, int val) { m[key] = val; }

  std::shared_ptr<monty::ndarray<long long,1>> keys()
  { 
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<long long,1>>(new monty::ndarray<long long,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->first;

    return res;    
  }

  std::shared_ptr<monty::ndarray<int,1>> values()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<int,1>>(new monty::ndarray<int,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->second;

    return res;
  }

  IntMap::t clone();
  IntMap::t __mosek_2fusion_2Utils_2IntMap__clone();
};



struct p_StringIntMap
{
  StringIntMap * _pubself;

  static p_StringIntMap * _get_impl(StringIntMap * _inst) { return _inst->_impl.get(); }

  p_StringIntMap(StringIntMap * _pubself) : _pubself(_pubself) {}

  static StringIntMap::t _new_StringIntMap() { return new StringIntMap(); }

  ::std::unordered_map<std::string,int> m;

  bool hasItem (const std::string & key) { return m.find(key) != m.end(); }
  int  getItem (const std::string & key) { return m.find(key)->second; } // will probably throw something or crash of no such key
  void setItem (const std::string & key, int val) { m[key] = val; }

  std::shared_ptr<monty::ndarray<std::string,1>> keys()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<std::string,1>>(new monty::ndarray<std::string,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->first;

    return res;
  }

  std::shared_ptr<monty::ndarray<int,1>> values()
  {
    size_t size = m.size();
    auto res = std::shared_ptr<monty::ndarray<int,1>>(new monty::ndarray<int,1>(monty::shape((int)size)));

    ptrdiff_t i = 0;
    for (auto it = m.begin(); it != m.end(); ++it)
      (*res)[i++] = it->second;

    return res;
  }

  StringIntMap::t clone();
  StringIntMap::t __mosek_2fusion_2Utils_2StringIntMap__clone();
};
// End of file 'src\fusion\cxx\IntMap_p.h'
// mosek.fusion.Utils.StringBuffer from file 'src\fusion\cxx\StringBuffer_p.h'
// namespace mosek::fusion::Utils
struct p_StringBuffer
{
  StringBuffer * _pubthis; 
  std::stringstream ss;

  p_StringBuffer(StringBuffer * _pubthis) : _pubthis(_pubthis) {}

  static p_StringBuffer * _get_impl(StringBuffer::t ptr) { return ptr->_impl.get(); }
  static p_StringBuffer * _get_impl(StringBuffer * ptr) { return ptr->_impl.get(); }

  static StringBuffer::t _new_StringBuffer() { return new StringBuffer(); }

  StringBuffer::t clear ();
  
  StringBuffer::t a (const monty::ndarray<std::string,1> & val);
  StringBuffer::t a (const monty::ndarray<int,1> & val);
  StringBuffer::t a (const monty::ndarray<long long,1> & val);
  StringBuffer::t a (const monty::ndarray<double,1> & val);
  

  StringBuffer::t a (const int & val);
  StringBuffer::t a (const long long & val);
  StringBuffer::t a (const double & val);
  StringBuffer::t a (const bool & val);
  StringBuffer::t a (const std::string & val);

  StringBuffer::t lf ();
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__clear ();

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<std::string,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<int,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<long long,1> & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const monty::ndarray<double,1> & val);

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const int & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const long long & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const double & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const bool & val);
  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__a (const std::string & val);

  StringBuffer::t __mosek_2fusion_2Utils_2StringBuffer__lf ();

  std::string toString () const;
};
// End of file 'src\fusion\cxx\StringBuffer_p.h'
}
}
}
#endif
