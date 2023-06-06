#
# A simple Python console for inspection of MOSEK tasks.
# For demonstration and simple debugging purposes.
#
# MOSEK ApS, 2018
#
import mosek, sys, re
import numpy as np 


#########################################################
# Global variables and initialization
def msk_write_log(text):
    sys.stdout.write(text)
    sys.stdout.flush()
    if logfile:
        logfile.write(text)
        logfile.flush()

def msk_print(text):
    msk_write_log(text + '\n')

env  = mosek.Env()
task = env.Task()
env.set_Stream(mosek.streamtype.log, msk_write_log)
task.set_Stream(mosek.streamtype.log, msk_write_log)
lastFile = None
logfile = None

welcometext = """
    *******           All in on cones!
 ************* 
***         ***       MOSEK Python Console, version {0}
****       ****       Type help for list of commands
*****     *****       Full documentation in the user manual
******   ******
 ****** ******        https://www.mosek.com
    *******           https://www.mosek.com/documentation
""".format('.'.join([str(_) for _ in mosek.Env.getversion()]))

helptext = [
    ('help [command]', 'Print list of commands or info about a specific command'),
    ('log filename', 'Save the session to a file'),
    ('intro', 'Print MOSEK splashscreen'),
    ('testlic', 'Test the license system'),
    ('read filename', 'Load problem from file'),
    ('reread', 'Reload last problem file'),
    ('solve [options]', 'Solve current problem'),
    ('write filename', 'Write current problem to file'),
    ('param [name [value]]', 'Set a parameter or get parameter values'),
    ('paramdef', 'Set all parameters to default values'),
    ('paramdiff', 'Show parameters with non-default values'),
    ('info [name]', 'Get an information item'),    
    ('anapro', 'Analyze problem data'),
    ('hist', 'Plot a histogram of problem data'),    
    ('histsol', 'Plot a histogram of the solutions'),    
    ('spy', 'Plot the sparsity pattern of the A matrix'),            
    ('truncate epsilon', 'Truncate small coefficients down to 0'),  
    ('resobj [fac]', 'Rescale objective by a factor'),
    ('anasol', 'Analyze solutions'),
    ('removeitg', 'Remove integrality constraints'),
    ('removecones', 'Remove all cones and leave just the linear part'),  
    ('delsol', 'Remove solutions'),
    ('infsub', 'Replace current problem with its infeasible subproblem'),
    ('writesol basename', 'Write solution(s) to file(s) with given basename'),
    ('optserver [url]', 'Use an OptServer to optimize'),
    ('exit', 'Leave')
]

helpdetail = {
    'help': 'help [command]\n\nPrint list of commands or info about a specific command.',
    'intro': 'intro\n\nPrint MOSEK splashscreen with into about version, license system settings etc.',
    'testlic': 'testlic\n\nPerform a test of the license system.',
    'read': 'read filename\n\nLoad a problem from a file in one of the formats supported by MOSEK. The file format is determined from the extension. It completely replaces any task that has been read previously.',
    'reread': 'reread\n\nLoad a problem from the same file as before.',
    'write': 'write filename\n\nWrite problem data to a file in one of the formats supported by MOSEK. The file format is determined from the extension. If the file format supports solutions, and the task contains them, they will also be written.',
    'param': 'param [name [value]]\n\nIf both arguments are given, sets a parameter whose generic name is \"name\" to the given value. The name may be incomplete, but it must match exactly one existing parameter.\nIf no value is given it lists the values of all parameters whose generic names contain the pattern \"name\".\nWith no arguments it lists the values of all parameters.\nIf no data file was yet read the values printed are the defaults in MOSEK.',
    'paramdef': 'paramdef\n\nSet all parameters to default values.',
    'paramdiff': 'paramdiff\n\nShow all parameters in the task which have values other than default.',
    'info': 'info [name]\n\nPrint the values of all task information items whose names contain the pattern \"name\". If no name is given it prints the values of all information items.',
    'anapro': 'anapro\n\nPrint structural and numerical statistics of the problem data, variables, constraints, cones, bounds, coefficients, etc.',
    'hist': 'hist\n\nPlot a simple histogram of the numerical part of the data.',
    'histsol': 'histsol\n\nPlot a simple histogram of the available solutions.',
    'spy': 'spy\n\nPlot the sparsity pattern of the A matrix.',    
    'truncate': 'truncate epsilon\n\nTruncate all numerical coefficients of absolute value less than epsilon down to 0.',
    'resobj': 'resobj [fac]\n\nRescale the objective by a factor of fac. Default is fac=0 (removing the objective).',
    'removeitg': 'removeitg\n\nChange all integer variables to continuous ones, i.e. construct the continuous relaxation of a given mixed-integer problem. The operation is non-reversible.',
    'removecones': 'removecones\n\nRemove all cones and semidefinite variables, leaving only the linear part of the problem.',
    'exit': 'exit\n\nLeave.',
    'writesol': 'writesol basename\n\nWrite the solutions to files basename.bas (basic), basename.sol (interior) and basename.itg (integer) for all solutions present in the task.',
    'solve': 'solve [options]\n\nSolve the problem.\nOptions are shorthands for typical actions useful when debugging and analyzing solver behavior. They simply set relevant MOSEK parameters, and these settings remain in force for this and subsequent optimizations or until changed. Each option has the format name=value, with different options separated by spaces. The recognized options are:\n\nalg=any|intpnt|psimplex|dsimplex|simplex|mio - Chooses optimizer. If \"any\" then MOSEK chooses.\nform=free|primal|dual - Solve primal or dualize. If \"free\" then MOSEK chooses.\npresolve=on|yes|off|no - Whether to use presolve.\ntime=_float_ - Time limit (seconds).\nmiogap=_float_ - Mixed-integer relative gap tolerance.\nsense=min|max - Minimize or maximize.\nthreads=_int_ - Number of threads.\ninfrep=on|yes|off|no - Whether to print an extended infeasibility report.\n\nExample: solve form=dual time=10.0',
    'anasol': 'anasol\n\nAnalyze the solution(s) and print a summary including objective values, status and violations.',
    'log': 'log filename\n\nSaves the entire session to a text file. The file is closed when the exit command is issued. Should only be used once.',
    'delsol': 'delsol\n\nRemove all solutions from the task.',
    'infsub':' infsub\n\nReplace the current problem by its infeasible subproblem (likely much smaller). Available for linear and conic problems.',
    'optserver': 'optserver [url]\n\nUse an OptServer running on the given URL to optimize. No url removes the redirection.',
}

# List of symbolic values for combinatorial parameters
# If this dictionary is empty all integer parameters will simply be displayed as integer values
# Otherwise those with symbolic names will be translated to generic string names 
combinatorial_parameters = {'MSK_IPAR_AUTO_UPDATE_SOL_INFO': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_REMOVE_UNUSED_SOLUTIONS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INTPNT_HOTSTART': {0: 'MSK_INTPNT_HOTSTART_NONE', 1: 'MSK_INTPNT_HOTSTART_PRIMAL', 2: 'MSK_INTPNT_HOTSTART_DUAL', 3: 'MSK_INTPNT_HOTSTART_PRIMAL_DUAL'}, 'MSK_IPAR_INTPNT_PURIFY': {0: 'MSK_PURIFY_NONE', 1: 'MSK_PURIFY_PRIMAL', 2: 'MSK_PURIFY_DUAL', 3: 'MSK_PURIFY_PRIMAL_DUAL', 4: 'MSK_PURIFY_AUTO'}, 'MSK_IPAR_INTPNT_MULTI_THREAD': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_LOG_INCLUDE_SUMMARY': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_LOG_LOCAL_INFO': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_BI_CLEAN_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_INTPNT_STARTING_POINT': {0: 'MSK_STARTING_POINT_FREE', 1: 'MSK_STARTING_POINT_GUESS', 2: 'MSK_STARTING_POINT_CONSTANT', 3: 'MSK_STARTING_POINT_SATISFY_BOUNDS'}, 'MSK_IPAR_INTPNT_DIFF_STEP': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INTPNT_SCALING': {0: 'MSK_SCALING_FREE', 1: 'MSK_SCALING_NONE', 2: 'MSK_SCALING_MODERATE', 3: 'MSK_SCALING_AGGRESSIVE'}, 'MSK_IPAR_INTPNT_SOLVE_FORM': {0: 'MSK_SOLVE_FREE', 1: 'MSK_SOLVE_PRIMAL', 2: 'MSK_SOLVE_DUAL'}, 'MSK_IPAR_INTPNT_ORDER_METHOD': {0: 'MSK_ORDER_METHOD_FREE', 1: 'MSK_ORDER_METHOD_APPMINLOC', 2: 'MSK_ORDER_METHOD_EXPERIMENTAL', 3: 'MSK_ORDER_METHOD_TRY_GRAPHPAR', 4: 'MSK_ORDER_METHOD_FORCE_GRAPHPAR', 5: 'MSK_ORDER_METHOD_NONE'}, 'MSK_IPAR_INTPNT_BASIS': {0: 'MSK_BI_NEVER', 1: 'MSK_BI_ALWAYS', 2: 'MSK_BI_NO_ERROR', 3: 'MSK_BI_IF_FEASIBLE', 4: 'MSK_BI_RESERVERED'}, 'MSK_IPAR_BI_IGNORE_MAX_ITER': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_BI_IGNORE_NUM_ERROR': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_PRESOLVE_USE': {0: 'MSK_PRESOLVE_MODE_OFF', 1: 'MSK_PRESOLVE_MODE_ON', 2: 'MSK_PRESOLVE_MODE_FREE'}, 'MSK_IPAR_PRESOLVE_LINDEP_USE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SIM_PRIMAL_SELECTION': {0: 'MSK_SIM_SELECTION_FREE', 1: 'MSK_SIM_SELECTION_FULL', 2: 'MSK_SIM_SELECTION_ASE', 3: 'MSK_SIM_SELECTION_DEVEX', 4: 'MSK_SIM_SELECTION_SE', 5: 'MSK_SIM_SELECTION_PARTIAL'}, 'MSK_IPAR_SIM_DUAL_SELECTION': {0: 'MSK_SIM_SELECTION_FREE', 1: 'MSK_SIM_SELECTION_FULL', 2: 'MSK_SIM_SELECTION_ASE', 3: 'MSK_SIM_SELECTION_DEVEX', 4: 'MSK_SIM_SELECTION_SE', 5: 'MSK_SIM_SELECTION_PARTIAL'}, 'MSK_IPAR_SIM_HOTSTART_LU': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_MODE': {0: 'MSK_MIO_MODE_IGNORED', 1: 'MSK_MIO_MODE_SATISFIED'}, 'MSK_IPAR_MIO_NODE_SELECTION': {0: 'MSK_MIO_NODE_SELECTION_FREE', 1: 'MSK_MIO_NODE_SELECTION_FIRST', 2: 'MSK_MIO_NODE_SELECTION_BEST', 3: 'MSK_MIO_NODE_SELECTION_PSEUDO'}, 'MSK_IPAR_MIO_BRANCH_DIR': {0: 'MSK_BRANCH_DIR_FREE', 1: 'MSK_BRANCH_DIR_UP', 2: 'MSK_BRANCH_DIR_DOWN', 3: 'MSK_BRANCH_DIR_NEAR', 4: 'MSK_BRANCH_DIR_FAR', 5: 'MSK_BRANCH_DIR_ROOT_LP', 6: 'MSK_BRANCH_DIR_GUIDED', 7: 'MSK_BRANCH_DIR_PSEUDOCOST'}, 'MSK_IPAR_MIO_ROOT_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_MIO_NODE_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_MIO_PERSPECTIVE_REFORMULATE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_PROPAGATE_OBJECTIVE_CONSTRAINT': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_CONIC_OUTER_APPROXIMATION': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_READ_KEEP_FREE_CON': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_READ_MPS_FORMAT': {0: 'MSK_MPS_FORMAT_STRICT', 1: 'MSK_MPS_FORMAT_RELAXED', 2: 'MSK_MPS_FORMAT_FREE', 3: 'MSK_MPS_FORMAT_CPLEX'}, 'MSK_IPAR_WRITE_MPS_FORMAT': {0: 'MSK_MPS_FORMAT_STRICT', 1: 'MSK_MPS_FORMAT_RELAXED', 2: 'MSK_MPS_FORMAT_FREE', 3: 'MSK_MPS_FORMAT_CPLEX'}, 'MSK_IPAR_READ_DEBUG': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_READ_LP_QUOTED_NAMES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_DATA_PARAM': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_FREE_CON': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_GENERIC_NAMES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_MPS_INT': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_LP_STRICT_FORMAT': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_LP_QUOTED_NAMES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_LP_FULL_OBJ': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_SOL_IGNORE_INVALID_NAMES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_SOL_HEAD': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_SOL_CONSTRAINTS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_SOL_VARIABLES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_SOL_BARVARIABLES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_BAS_HEAD': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_BAS_CONSTRAINTS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_BAS_VARIABLES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_INT_HEAD': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_INT_CONSTRAINTS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_INT_VARIABLES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INFEAS_REPORT_AUTO': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INFEAS_GENERIC_NAMES': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_LICENSE_WAIT': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_LICENSE_SUPPRESS_EXPIRE_WRNS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_LICENSE_DEBUG': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SOL_FILTER_KEEP_BASIC': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SOL_FILTER_KEEP_RANGED': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_READ_TASK_IGNORE_PARAM': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_TASK_INC_SOL': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_PARAM_READ_CASE_NAME': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_PARAM_READ_IGN_ERROR': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SOLUTION_CALLBACK': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SIM_SCALING': {0: 'MSK_SCALING_FREE', 1: 'MSK_SCALING_NONE', 2: 'MSK_SCALING_MODERATE', 3: 'MSK_SCALING_AGGRESSIVE'}, 'MSK_IPAR_SIM_SCALING_METHOD': {0: 'MSK_SCALING_METHOD_POW2', 1: 'MSK_SCALING_METHOD_FREE'}, 'MSK_IPAR_SIM_HOTSTART': {0: 'MSK_SIM_HOTSTART_NONE', 1: 'MSK_SIM_HOTSTART_FREE', 2: 'MSK_SIM_HOTSTART_STATUS_KEYS'}, 'MSK_IPAR_SIM_BASIS_FACTOR_USE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SIM_DEGEN': {0: 'MSK_SIM_DEGEN_NONE', 1: 'MSK_SIM_DEGEN_FREE', 2: 'MSK_SIM_DEGEN_AGGRESSIVE', 3: 'MSK_SIM_DEGEN_MODERATE', 4: 'MSK_SIM_DEGEN_MINIMUM'}, 'MSK_IPAR_SIM_REFORMULATION': {1: 'MSK_SIM_REFORMULATION_ON', 0: 'MSK_SIM_REFORMULATION_OFF', 2: 'MSK_SIM_REFORMULATION_FREE', 3: 'MSK_SIM_REFORMULATION_AGGRESSIVE'}, 'MSK_IPAR_SIM_EXPLOIT_DUPVEC': {1: 'MSK_SIM_EXPLOIT_DUPVEC_ON', 0: 'MSK_SIM_EXPLOIT_DUPVEC_OFF', 2: 'MSK_SIM_EXPLOIT_DUPVEC_FREE'}, 'MSK_IPAR_SIM_SAVE_LU': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SIM_NON_SINGULAR': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INFEAS_PREFER_PRIMAL': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_READ_LP_DROP_NEW_VARS_IN_BOU': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_HINTS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_PARAMETERS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_PROBLEM': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_HEADER': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_SOLUTIONS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_SOL_BAS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_SOL_ITG': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_OPF_WRITE_SOL_ITR': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_PTF_WRITE_TRANSFORM': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_PRIMAL_REPAIR_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_MIO_CUT_CMIR': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_CUT_CLIQUE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_CUT_IMPLIED_BOUND': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_CUT_KNAPSACK_COVER': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_MIO_CUT_GMI': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SENSITIVITY_TYPE': {0: 'MSK_SENSITIVITY_TYPE_BASIS'}, 'MSK_IPAR_SENSITIVITY_ALL': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_CACHE_LICENSE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_INTPNT_REGULARIZATION_USE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SIM_SOLVE_FORM': {0: 'MSK_SOLVE_FREE', 1: 'MSK_SOLVE_PRIMAL', 2: 'MSK_SOLVE_DUAL'}, 'MSK_IPAR_SIM_SWITCH_OPTIMIZER': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_ITEMS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_CHECK_CONVEXITY': {0: 'MSK_CHECK_CONVEXITY_NONE', 1: 'MSK_CHECK_CONVEXITY_SIMPLE', 2: 'MSK_CHECK_CONVEXITY_FULL'}, 'MSK_IPAR_AUTO_SORT_A_BEFORE_OPT': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_SENSITIVITY_OPTIMIZER': {2: 'MSK_OPTIMIZER_FREE', 4: 'MSK_OPTIMIZER_INTPNT', 0: 'MSK_OPTIMIZER_CONIC', 6: 'MSK_OPTIMIZER_PRIMAL_SIMPLEX', 1: 'MSK_OPTIMIZER_DUAL_SIMPLEX', 3: 'MSK_OPTIMIZER_FREE_SIMPLEX', 5: 'MSK_OPTIMIZER_MIXED_INT'}, 'MSK_IPAR_WRITE_XML_MODE': {0: 'MSK_WRITE_XML_MODE_ROW', 1: 'MSK_WRITE_XML_MODE_COL'}, 'MSK_IPAR_ANA_SOL_BASIS': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_ANA_SOL_PRINT_VIOLATED': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_BASIS_SOLVE_USE_PLUS_ONE': {1: 'MSK_ON', 0: 'MSK_OFF'}, 'MSK_IPAR_COMPRESS_STATFILE': {1: 'MSK_ON', 0: 'MSK_OFF'}}


msk_print(welcometext)


#########################################################
# Pretty printing and the like
def msk_formatline(width, parts):
    msk_print(''.join(['{0:<{width}}'.format(p, width=width) for p in parts]).rstrip())

def msk_togeneric(pythonname):
    subs = { 'iparam' : 'MSK_IPAR',
             'dparam' : 'MSK_DPAR',
             'sparam' : 'MSK_SPAR',
             'iinfitem' : 'MSK_IINF',
             'dinfitem' : 'MSK_DINF',
             'liinfitem' : 'MSK_LIINF',}
    t = pythonname.split('.')
    if len(t) == 1:
        return t[0].upper()
    else:
        return subs[t[0]] + '_' + t[1].upper()

def msk_responsecode(res):
    code, desc = mosek.Env.getcodedesc(res)
    return '{0} ({1}) {2}'.format(code, int(res), desc)



#########################################################
# Simple one-line operations

# Print help
def msk_help():
    for cmd,desc in helptext:
        msk_formatline(30, [cmd, desc])

def msk_help_detail(cmd):
    if cmd in helpdetail: msk_print(helpdetail[cmd])
    else: msk_print("Unknown command")

# Print intro
def msk_intro():
    env.echointro(1)

# Load task from a file
def msk_read(fname):
    global lastFile
    task.readdata(fname)
    lastFile = fname

# Load data from the same task as before
def msk_reread():
    if lastFile: msk_read(lastFile)

# Write data to file
def msk_write(fname):
    task.writedata(fname)

# Open a log file if there is none yet
def msk_log(fname):
    global logfile
    if logfile is None:
        logfile = open(fname, 'w')

# Write all solutions
def msk_writesol(name):
    ext = { mosek.soltype.bas: 'bas',
            mosek.soltype.itg: 'itg',
            mosek.soltype.itr: 'sol' }
    for stype in mosek.soltype.values:
        if task.solutiondef(stype):
            task.writesolution(stype, name + '.' + ext[stype])

# Remove integrality constraints
def msk_removeitg():
    numvar = task.getnumvar()
    task.putvartypelist(list(range(numvar)), [mosek.variabletype.type_cont]*numvar)

# Remove cones
def msk_removecones():
    numbarvar = task.getnumbarvar()
    if numbarvar > 0:
        task.removebarvars(list(reversed(range(numbarvar))))
    numcone = task.getnumcone()
    if numcone > 0:
        task.removecones(list(reversed(range(numcone))))

# Remove all solutions
def msk_delsol():
    for sol in mosek.soltype.values:
        task.deletesolution(sol)

# Replace with an infeasible subproblem
def msk_infsub():
    try:
        global task
        newtask = None
        if task.solutiondef(mosek.soltype.bas):
            newtask = task.getinfeasiblesubproblem(mosek.soltype.bas)
        elif task.solutiondef(mosek.soltype.itr):
            newtask = task.getinfeasiblesubproblem(mosek.soltype.itr)
        if newtask:
            task.__del__()
            task = newtask
            task.set_Stream(mosek.streamtype.log, msk_write_log)
    except:
        print("Operation not possible")


# Set a parameter (generic name and string value)
def msk_putparam(name, val):
    # Find names that match the given string
    # If some matches EXACTLY then we take it immediately
    # Otherwise make sure there is only one match
    matches = []
    for param in mosek.iparam.values + mosek.dparam.values + mosek.sparam.values:
        genname = msk_togeneric(param.__str__())
        if name == genname:
            task.putparam(name, val)
            return
        if re.search(name, genname):
            matches.append(genname)
    if len(matches) == 0: 
        msk_print('No matching parameter')
    elif len(matches) > 1:
        msk_print('Too many matching parameters:')
        for genname in matches:
            msk_print(genname)
    else:
        task.putparam(matches[0], val)

# Get parameter values for parameter names matching pattern
def msk_getparam(name):
    ptypes = { mosek.iparam: 'getintparam', 
               mosek.dparam: 'getdouparam',
               mosek.sparam: 'getstrparam',}
    for ptype in ptypes:
        func = getattr(task, ptypes[ptype])
        for param in ptype.values:
            genname = msk_togeneric(param.__str__())
            if re.search(name, genname):
                val = func(param)
                if genname in combinatorial_parameters and val in combinatorial_parameters[genname]:
                    val = combinatorial_parameters[genname][val]
                msk_formatline(55, [genname, str(val)])

# Set default parameter values
def msk_paramdef():
    task.setdefaults()

# Print parameters with values other than default
def msk_paramdiff():
    ptypes = { mosek.iparam: 'getintparam', 
               mosek.dparam: 'getdouparam',
               mosek.sparam: 'getstrparam',}
    msk_formatline(55, ["Name", "Value", "Default"])
    with env.Task() as deftask:
        for ptype in ptypes:
            func    = getattr(task,    ptypes[ptype])
            deffunc = getattr(deftask, ptypes[ptype])
            for param in ptype.values:
                genname = msk_togeneric(param.__str__())
                val, defval = func(param), deffunc(param)
                if val != defval:
                    if genname in combinatorial_parameters:
                        val    = combinatorial_parameters[genname][val]
                        defval = combinatorial_parameters[genname][defval]
                    msk_formatline(55, [genname, str(val), str(defval)])

# Get information items
def msk_getinfo(name):
    itypes = { mosek.iinfitem: 'getintinf', 
               mosek.dinfitem: 'getdouinf',
               mosek.liinfitem: 'getlintinf',}
    for itype in itypes:
        func = getattr(task, itypes[itype])
        for item in itype.values:
            genname = msk_togeneric(item.__str__())
            if re.search(name, genname):
                val = func(item)
                msk_formatline(55, [genname, str(val)])

# Test license system
def msk_testlic():
    env.putlicensedebug(1)
    env.echointro(1)
    for feat in [mosek.feature.pts, mosek.feature.pton]:
        msk_print('')
        try:
            env.checkoutlicense(feat)
            env.checkinlicense(feat)
            msk_print('** Feature ' + msk_togeneric(feat.__name__) + ': OK')
        except mosek.Error as e:
            msk_print('** Feature ' + msk_togeneric(feat.__name__) + ': ERROR')
            msk_print('Error code: {0}'.format(msk_responsecode(e.errno)))
    env.checkinall()
    env.putlicensedebug(0)

# Finish
def msk_exit():
    task.__del__()
    env.__del__()
    if logfile: logfile.close()
    sys.exit(0)

# Extra options for solve
def msk_process_solve_opts(opts):
    def isfloat(x):
        try:
            float(x)
            return True
        except:
            return False

    parnames  = {'alg'      : ['MSK_IPAR_OPTIMIZER'],
                 'form'     : ['MSK_IPAR_INTPNT_SOLVE_FORM', 'MSK_IPAR_SIM_SOLVE_FORM'],
                 'presolve' : ['MSK_IPAR_PRESOLVE_USE'],
                 'time'     : ['MSK_DPAR_MIO_MAX_TIME', 'MSK_DPAR_OPTIMIZER_MAX_TIME'],
                 'miogap'   : ['MSK_DPAR_MIO_TOL_REL_GAP'],
                 'threads'  : ['MSK_IPAR_NUM_THREADS'],
                 'infrep'   : ['MSK_IPAR_INFEAS_REPORT_AUTO'],
                }
    parvalues = {'yes'      : 'MSK_ON',
                 'no'       : 'MSK_OFF',
                 'on'       : 'MSK_ON',
                 'off'      : 'MSK_OFF',
                 'psimplex' : 'MSK_OPTIMIZER_PRIMAL_SIMPLEX',
                 'dsimplex' : 'MSK_OPTIMIZER_DUAL_SIMPLEX',
                 'simplex'  : 'MSK_OPTIMIZER_FREE_SIMPLEX',
                 'intpnt'   : 'MSK_OPTIMIZER_CONIC',
                 'mio'      : 'MSK_OPTIMIZER_MIXED_INT',
                 'any'      : 'MSK_OPTIMIZER_FREE',
                 'primal'   : 'MSK_SOLVE_PRIMAL',
                 'dual'     : 'MSK_SOLVE_DUAL',
                 'free'     : 'MSK_SOLVE_FREE',
                 }
    for o in opts:
        name, val = o.split('=')
        if (name in parnames) and (val in parvalues):
            for parname in parnames[name]:
                task.putparam(parname, parvalues[val])
        elif (name in parnames) and isfloat(val):
            for parname in parnames[name]:
                task.putparam(parname, val)
        elif name == 'sense' and val == 'min':
            task.putobjsense(mosek.objsense.minimize)
        elif name == 'sense' and val == 'max':
            task.putobjsense(mosek.objsense.maximize)
        else:
            msk_print('WARNING: Unknown option ' + name + '=' + val)

# Solve
def msk_solve():
    # Set some parameters useful for debugging
    task.putintparam(mosek.iparam.log_cut_second_opt, 0)
    task.putintparam(mosek.iparam.cache_license, mosek.onoffkey.off)
    task.putintparam(mosek.iparam.auto_update_sol_info, mosek.onoffkey.on)
    task.putintparam(mosek.iparam.infeas_report_level, 10)
    try:
        res = task.optimize()
        task.solutionsummary(mosek.streamtype.log)
        msk_print('\nTermination code: {0}'.format(msk_responsecode(res)))
    except mosek.Error as e:
        msk_print('Error code: {0}'.format(msk_responsecode(e.errno)))


# Analyze the problem structure - variables
def msk_anapro_struct_var():
    # Number of variables
    numvar = task.getnumvar()
    numintvar = task.getnumintvar()
    numbarvar = task.getnumbarvar()
    msk_print('** Variables')
    msk_formatline(18, ['scalar: {0}'.format(numvar),
                        'integer: {0}'.format(numintvar),
                        'matrix: {0}'.format(numbarvar)])

    # Number of variables with various bounds
    if numvar > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numvar, [0.0]*numvar, [0.0]*numvar
        task.getvarboundslice(0, numvar, bk, bl, bu)
        stat = { key: 0 for key in mosek.boundkey.values }
        for i in range(numvar): stat[bk[i]] += 1
        msk_formatline(18, ['low: {0}'.format(stat[mosek.boundkey.lo]),
                            'up: {0}'.format(stat[mosek.boundkey.up]),
                            'ranged: {0}'.format(stat[mosek.boundkey.ra]),
                            'free: {0}'.format(stat[mosek.boundkey.fr]),
                            'fixed: {0}'.format(stat[mosek.boundkey.fx])])

    # Number of integer variables
    if numintvar >0:
        vtype = [mosek.variabletype.type_cont]*numvar
        task.getvartypelist(list(range(0,numvar)), vtype)
        numbinvar = 0
        for i in range(numvar):
            if vtype[i] == mosek.variabletype.type_int and bk[i] == mosek.boundkey.ra and bl[i] == 0.0 and bu[i] == 1.0:
                numbinvar += 1
        msk_formatline(18, ['binary: {0}'.format(numbinvar),
                            'gen. int.: {0}'.format(numintvar-numbinvar)])

    msk_print('')

# Analyze the problem structure - constraints
def msk_anapro_struct_con():
    # Number of constraints
    numcon = task.getnumcon()
    msk_print('** Constraints')
    msk_formatline(18, ['all: {0}'.format(numcon)])

    # Number of constraints with various bounds
    if numcon > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numcon, [0.0]*numcon, [0.0]*numcon
        task.getconboundslice(0, numcon, bk, bl, bu)
        stat = { key: 0 for key in mosek.boundkey.values }
        for i in range(numcon): stat[bk[i]] += 1
        msk_formatline(18, ['low: {0}'.format(stat[mosek.boundkey.lo]),
                            'up: {0}'.format(stat[mosek.boundkey.up]),
                            'ranged: {0}'.format(stat[mosek.boundkey.ra]),
                            'free: {0}'.format(stat[mosek.boundkey.fr]),
                            'fixed: {0}'.format(stat[mosek.boundkey.fx])])
    msk_print('')


# Analyze the problem structure - cones
def msk_anapro_struct_cones():
    numcones = task.getnumcone()
    numbarvar = task.getnumbarvar()
    if numcones + numbarvar > 0:
        msk_print('** Cones')
        stat = { ct : {} for ct in mosek.conetype.values }
        for i in range(numcones):
            ct, cpar, nummem = task.getconeinfo(i)
            if nummem in stat[ct]: stat[ct][nummem] += 1
            else: stat[ct][nummem] = 1
        for ct in stat:
            if stat[ct]:
                msk_formatline(18, [msk_togeneric(ct.__name__) + ': ' + str(sum([stat[ct][d] for d in stat[ct]])),
                                    'dims: ' + ', '.join(['{0}: {1}'.format(d,stat[ct][d]) for d in sorted(stat[ct])]) ])

        # Statistics of PSD variables
        if numbarvar > 0:
            dims = {}
            for i in range(numbarvar):
                d = task.getdimbarvarj(i)
                if d in dims: dims[d] += 1
                else: dims[d] = 1
            msk_formatline(18, ['matrix: ' + str(numbarvar), 
                                'dims: ' + ', '.join(['{0}: {1}'.format(d,dims[d]) for d in sorted(dims)]) ])

        msk_print('')

# Return number of nonzeros, min, max absolute value of nonzeros for a vector
def msk_ana_vector_nnz(v):
    vmin, vmax = 0.0, 0.0
    vnnz = [ abs(x) for x in v if x != 0 ]
    nnz = len(vnnz)
    if nnz > 0:
        vmin, vmax = min(vnnz), max(vnnz)
    return nnz, vmin, vmax

# Return number of elements, min, max for a vector
def msk_ana_vector(v):
    if len(v) > 0:
        return len(v), min(v), max(v)
    else:
        return 0, 0.0, 0.0

# Analyze problem data
def msk_anapro_data():
    numvar = task.getnumvar()
    numcon = task.getnumcon()
    msk_print('** Problem data (numerics)')

    # c
    if numvar > 0:
        c = [0.0] * numvar
        task.getc(c)
        cnnz, cmin, cmax = msk_ana_vector_nnz(c)
        if cnnz > 0: msk_formatline(18, ['|c|', 'nnz: {0}'.format(cnnz), 'min={0:.2e}'.format(cmin), 'max={0:.2e}'.format(cmax)])

    # A matrix
    if numcon > 0 and numvar > 0:
        annz = task.getnumanz64()
        ai, aj, av = [0]*annz, [0]*annz, [0.0]*annz
        task.getacolslicetrip(0, numvar, ai, aj, av)
        annz, amin, amax = msk_ana_vector_nnz(av)
        if annz > 0: msk_formatline(18, ['|A|', 'nnz: {0}'.format(annz), 'min={0:.2e}'.format(amin), 'max={0:.2e}'.format(amax)])    

    # bounds
    if numvar > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numvar, [0.0]*numvar, [0.0]*numvar
        task.getvarboundslice(0, numvar, bk, bl, bu)
        blxnnz, blxmin, blxmax = msk_ana_vector([bl[i] for i in range(numvar) if bk[i] in [mosek.boundkey.lo, mosek.boundkey.ra, mosek.boundkey.fx]])
        if blxnnz > 0: msk_formatline(18, ['blx', 'fin: {0}'.format(blxnnz), 'min={0:.2e}'.format(blxmin), 'max={0:.2e}'.format(blxmax)])    
        buxnnz, buxmin, buxmax = msk_ana_vector([bu[i] for i in range(numvar) if bk[i] in [mosek.boundkey.up, mosek.boundkey.ra, mosek.boundkey.fx]])
        if buxnnz > 0: msk_formatline(18, ['bux', 'fin: {0}'.format(buxnnz), 'min={0:.2e}'.format(buxmin), 'max={0:.2e}'.format(buxmax)])    
    if numcon > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numcon, [0.0]*numcon, [0.0]*numcon
        task.getconboundslice(0, numcon, bk, bl, bu)
        blcnnz, blcmin, blcmax = msk_ana_vector([bl[i] for i in range(numcon) if bk[i] in [mosek.boundkey.lo, mosek.boundkey.ra, mosek.boundkey.fx]])
        if blcnnz > 0: msk_formatline(18, ['blc', 'fin: {0}'.format(blcnnz), 'min={0:.2e}'.format(blcmin), 'max={0:.2e}'.format(blcmax)])    
        bucnnz, bucmin, bucmax = msk_ana_vector([bu[i] for i in range(numcon) if bk[i] in [mosek.boundkey.up, mosek.boundkey.ra, mosek.boundkey.fx]])
        if bucnnz > 0: msk_formatline(18, ['buc', 'fin: {0}'.format(bucnnz), 'min={0:.2e}'.format(bucmin), 'max={0:.2e}'.format(bucmax)])    

    # Qo
    qonnz = task.getnumqobjnz()
    if qonnz > 0:
        qi, qj, qv = [0]*qonnz, [0]*qonnz, [0.0]*qonnz
        task.getqobj(qi, qj, qv)
        qonnz, qomin, qomax = msk_ana_vector_nnz(qv)
        msk_formatline(18, ['|Qo|', 'nnz: {0}'.format(qonnz), 'min={0:.2e}'.format(qomin), 'max={0:.2e}'.format(qomax)])    

    # Qk
    qknnz = 0
    qkmin, qkmax = [], []
    for k in range(numcon):
        nnz = task.getnumqconknz(k)
        if nnz > 0:
            qi, qj, qv = [0]*nnz, [0]*nnz, [0.0]*nnz
            task.getqconk(k, qi, qj, qv)
            nnz, kmin, kmax = msk_ana_vector_nnz(qv)
            qkmin.append(kmin)
            qkmax.append(kmax)
            qknnz += nnz
    if qknnz > 0:
        msk_formatline(18, ['|Qk|', 'nnz: {0}'.format(qknnz), 'min={0:.2e}'.format(min(qkmin)), 'max={0:.2e}'.format(max(qkmax))]) 

    # barC
    barcnnz = task.getnumbarcblocktriplets()
    if barcnnz > 0:
        ci, cj, ck, cv, = [0]*barcnnz, [0]*barcnnz, [0]*barcnnz, [0.0]*barcnnz
        barcnnz = task.getbarcblocktriplet(ci, cj, ck, cv)
        barcnnz, barcmin, barcmax = msk_ana_vector_nnz(cv[:barcnnz])
        msk_formatline(18, ['|barC|', 'nnz: {0}'.format(barcnnz), 'min={0:.2e}'.format(barcmin), 'max={0:.2e}'.format(barcmax)])    
   
    # barA
    barannz = task.getnumbarablocktriplets()
    if barannz > 0:
        ai, aj, ak, al, av, = [0]*barannz, [0]*barannz, [0]*barannz, [0]*barannz, [0.0]*barannz
        barannz = task.getbarablocktriplet(ai, aj, ak, al, av)
        barannz, baramin, baramax = msk_ana_vector_nnz(av[:barannz])
        msk_formatline(18, ['|barA|', 'nnz: {0}'.format(barannz), 'min={0:.2e}'.format(baramin), 'max={0:.2e}'.format(baramax)])    

    msk_print('')

# Plot a simple histogram of the numerical data
def msk_hist():
    import matplotlib.pyplot as plt
    numvar = task.getnumvar()
    numcon = task.getnumcon()
    msk_hist.plotnum = 1
    msk_hist.numrow, msk_hist.numcol = 3, 3
    if task.getnumqobjnz() == 0 and sum(task.getnumqconknz(k) for k in range(numcon)) == 0 and task.getnumbarcblocktriplets() == 0 and task.getnumbarablocktriplets() == 0:
        msk_hist.numrow = 2

    def nextplot(name, data):
        plt.subplot(msk_hist.numrow, msk_hist.numcol, msk_hist.plotnum)
        plt.hist(data, bins=20)
        plt.title(name)
        msk_hist.plotnum += 1

    # c
    if numvar > 0:
        c = np.empty(numvar, dtype=float)
        task.getc(c)
        nextplot('c', c[np.where(c != 0)])

    # A matrix
    if numcon > 0 and numvar > 0:
        annz = task.getapiecenumnz(0, numcon, 0, numvar)
        if annz > 0:
            ai, aj, av = np.empty(annz, dtype=np.int32), np.empty(annz, dtype=np.int32), np.empty(annz, dtype=float)
            task.getacolslicetrip(0, numvar, ai, aj, av)
            nextplot('A', av)

    # bounds
    if numvar > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numvar, [0.0]*numvar, [0.0]*numvar
        task.getvarboundslice(0, numvar, bk, bl, bu)
        blxvec = [bl[i] for i in range(numvar) if bk[i] in [mosek.boundkey.lo, mosek.boundkey.ra, mosek.boundkey.fx]]
        if len(blxvec) > 0: nextplot('blx', np.array(blxvec))
        buxvec = [bu[i] for i in range(numvar) if bk[i] in [mosek.boundkey.up, mosek.boundkey.ra, mosek.boundkey.fx]]
        if len(buxvec) > 0: nextplot('bux', np.array(buxvec))

    if numcon > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numcon, [0.0]*numcon, [0.0]*numcon
        task.getconboundslice(0, numcon, bk, bl, bu)
        blcvec = [bl[i] for i in range(numcon) if bk[i] in [mosek.boundkey.lo, mosek.boundkey.ra, mosek.boundkey.fx]]
        if len(blcvec) > 0: nextplot('blc', np.array(blcvec))
        bucvec = [bu[i] for i in range(numcon) if bk[i] in [mosek.boundkey.up, mosek.boundkey.ra, mosek.boundkey.fx]]
        if len(bucvec) > 0: nextplot('buc', np.array(bucvec))

    # Qo
    qonnz = task.getnumqobjnz()
    if qonnz > 0:
        qi, qj, qv = np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=float)
        task.getqobj(qi, qj, qv)
        nextplot('Qo', qv)

    # Qk
    qknnz = 0
    qkall = np.empty([0], dtype=float)
    for k in range(numcon):
        nnz = task.getnumqconknz(k)
        if nnz > 0:
            qi, qj, qv = np.empty(nnz, dtype=np.int32), np.empty(nnz, dtype=np.int32), np.empty(nnz, dtype=float)
            task.getqconk(k, qi, qj, qv)
            qkall = np.append(qkall, qv)
            qknnz += nnz
    if qknnz > 0: nextplot('Qk', qkall)

    # barC
    barcnnz = task.getnumbarcblocktriplets()
    if barcnnz > 0:
        ci, cj, ck, cv = np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=float)
        nextplot('barC', cv)

    # barA
    barannz = task.getnumbarablocktriplets()
    if barannz > 0:
        ai, aj, ak, al, av = np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=float)
        barannz = task.getbarablocktriplet(ai, aj, ak, al, av)
        nextplot('barA', av)

    plt.show()

# Truncate all coefficients with abs <= epsilon down to zero
def msk_truncate(epsilon):
    def msk_slash(v):
        for i in range(len(v)):
            if abs(v[i]) < epsilon: v[i] = 0

    numvar = task.getnumvar()
    numcon = task.getnumcon()

    # c
    if numvar > 0:
        c = np.empty(numvar, dtype=float)
        task.getc(c)
        msk_slash(c)
        task.putclist(range(numvar), c)

    # A matrix
    if numcon > 0 and numvar > 0:
        annz = task.getapiecenumnz(0, numcon, 0, numvar)
        if annz > 0:
            ai, aj, av = np.empty(annz, dtype=np.int32), np.empty(annz, dtype=np.int32), np.empty(annz, dtype=float)
            task.getacolslicetrip(0, numvar, ai, aj, av)
            msk_slash(av)
            task.putaijlist(ai, aj, av)

    # bounds
    if numvar > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numvar, [0.0]*numvar, [0.0]*numvar
        task.getvarboundslice(0, numvar, bk, bl, bu)
        msk_slash(bl)
        msk_slash(bu)
        task.putvarboundslice(0, numvar, bk, bl, bu)

    if numcon > 0:
        bk, bl, bu = [mosek.boundkey.lo]*numcon, [0.0]*numcon, [0.0]*numcon
        task.getconboundslice(0, numcon, bk, bl, bu)
        msk_slash(bl)
        msk_slash(bu)
        task.putconboundslice(0, numcon, bk, bl, bu)

    # Qo
    qonnz = task.getnumqobjnz()
    if qonnz > 0:
        qi, qj, qv = np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=float)
        task.getqobj(qi, qj, qv)
        msk_slash(qv)
        task.putqobj(qi, qj, qv)

    # Qk
    for k in range(numcon):
        nnz = task.getnumqconknz(k)
        if nnz > 0:
            qi, qj, qv = np.empty(nnz, dtype=np.int32), np.empty(nnz, dtype=np.int32), np.empty(nnz, dtype=float)
            task.getqconk(k, qi, qj, qv)
            msk_slash(qv)
            task.putqconk(k, qi, qj, qv)

    # barC
    barcnnz = task.getnumbarcblocktriplets()
    if barcnnz > 0:
        cj, ck, cl, cv = np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=float)
        barcnnz = task.getbarcblocktriplet(cj, ck, cl, cv)
        msk_slash(cv)
        task.putbarcblocktriplet(barcnnz, cj, ck, cl, cv)

    # barA
    barannz = task.getnumbarablocktriplets()
    if barannz > 0:
        ai, aj, ak, al, av = np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=np.int32), np.empty(barannz, dtype=float)
        barannz = task.getbarablocktriplet(ai, aj, ak, al, av)
        msk_slash(av)
        task.putbarablocktriplet(barannz, ai, aj, ak, al, av)


# Rescale objective function
def msk_resobj(fac):
    numvar = task.getnumvar()

    # c
    if numvar > 0:
        c = np.empty(numvar, dtype=float)
        task.getc(c)
        task.putclist(range(numvar), c * fac)

    # Qo
    qonnz = task.getnumqobjnz()
    if qonnz > 0:
        qi, qj, qv = np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=np.int32), np.empty(qonnz, dtype=float)
        task.getqobj(qi, qj, qv)
        task.putqobj(qi, qj, qv * fac)

    # cfix
    cfix = task.getcfix()
    task.putcfix(cfix * fac)

    # barC
    barcnnz = task.getnumbarcblocktriplets()
    if barcnnz > 0:
        cj, ck, cl, cv = np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=np.int32), np.empty(barcnnz, dtype=float)
        barcnnz = task.getbarcblocktriplet(cj, ck, cl, cv)
        task.putbarcblocktriplet(barcnnz, cj, ck, cl, cv * fac)


# Plot the sparsity pattern of the A matrix
def msk_spy():
    import matplotlib.pyplot as plt
    import scipy.sparse as sp
    numvar = task.getnumvar()
    numcon = task.getnumcon()

    if numcon > 0 and numvar > 0:
        annz = task.getapiecenumnz(0, numcon, 0, numvar)
        if annz > 0:
            ai, aj, av = np.empty(annz, dtype=np.int32), np.empty(annz, dtype=np.int32), np.empty(annz, dtype=float)
            task.getacolslicetrip(0, numvar, ai, aj, av)
            plt.spy(sp.coo_matrix((av, (ai, aj))))
            plt.show()

# Append together a full primal solution
def msk_full_primal(stype):
    res = np.empty([0], dtype=float)

    # Scalar variables
    numvar = task.getnumvar()
    if numvar > 0:
        xx = np.empty(numvar, dtype=float)
        task.getxx(stype, xx)
        res = np.append(res, xx)

    # Matrix variables
    numbarvar = task.getnumbarvar()
    if numbarvar > 0:
        barlentot = sum(task.getlenbarvarj(j) for j in range(numbarvar))
        barx = np.empty(barlentot, dtype=float)
        task.getbarxslice(stype, 0, numbarvar, barlentot, barx)
        res = np.append(res, barx)

    return res

# Append together a full dual solution
def msk_full_dual(stype):
    res = np.empty([0], dtype=float)
    what = []

    # Linear part
    if task.getnumvar() > 0: what.extend([(task.getsux, task.getnumvar()), (task.getslx, task.getnumvar())])
    if task.getnumcon() > 0: what.extend([(task.getsuc, task.getnumcon()), (task.getslc, task.getnumcon())])
    if task.getnumcone() > 0: what.extend([(task.getsnx, task.getnumvar())])

    for getsfun, size in what:
        s = np.empty(size, dtype=float)
        getsfun(stype, s)
        res = np.append(res, s)

    # Matrix duals
    numbarvar = task.getnumbarvar()
    if numbarvar > 0:
        barlentot = sum(task.getlenbarvarj(j) for j in range(numbarvar))
        bars = np.empty(barlentot, dtype=float)
        task.getbarsslice(stype, 0, numbarvar, barlentot, bars)
        res = np.append(res, bars)

    return res

# Plot a simple histogram of the available solutions
def msk_histsol():
    import matplotlib.pyplot as plt
    msk_histsol.numrow, msk_histsol.numcol = sum(int(task.solutiondef(stype)) for stype in mosek.soltype.values), 2
    msk_histsol.plotnum = 1

    def nextplot(name, stype, data):
        plt.subplot(msk_histsol.numrow, msk_histsol.numcol, msk_histsol.plotnum)
        plt.hist(data, bins=20)
        plt.title("{0}({1})".format(name, str(stype).split('.')[1].upper()))

    if msk_histsol.numrow > 0:
        for stype in mosek.soltype.values:
            if task.solutiondef(stype):
                prosta = task.getprosta(stype)
                # Primal solution, if available
                if prosta not in [ mosek.prosta.prim_infeas, mosek.prosta.prim_and_dual_infeas ]:
                    nextplot('Primal', stype, msk_full_primal(stype))
                msk_histsol.plotnum += 1
                # Dual solution, if available
                if prosta not in [ mosek.prosta.dual_infeas, mosek.prosta.prim_and_dual_infeas ] and stype != mosek.soltype.itg:
                    nextplot('Dual', stype, msk_full_dual(stype))
                msk_histsol.plotnum += 1

        plt.show()
    else:
        print('No solutions to analyze')

# Analyze the solution - print summary
def msk_anasol():
    task.solutionsummary(mosek.streamtype.log)
    for stype in mosek.soltype.values:
        if task.solutiondef(stype):
            task.analyzesolution(mosek.streamtype.log, stype)

# Set an OptServer URL
def msk_optserver(url):
    task.putoptserverhost(url)

####################################################################
# Execute a single command
def msk_command(l):
    if len(l)>0:
        cmd = l.pop(0)

        try:
            if cmd == 'exit': 
                msk_exit()
            elif cmd == 'log':
                if len(l) > 0: msk_log(l[0])
                else: msk_print('Missing filename')
            elif cmd == 'solve':
                msk_process_solve_opts(l)
                msk_solve()
            elif cmd == 'help':
                if len(l) > 0: msk_help_detail(l[0])
                else: msk_help()
            elif cmd == 'reread':
                msk_reread()
            elif cmd == 'intro':
                msk_intro()
            elif cmd == 'testlic':
                msk_testlic()
            elif cmd == 'removeitg':
                msk_removeitg()
            elif cmd == 'removecones':
                msk_removecones()
            elif cmd == 'read':
                if len(l) > 0: msk_read(l[0])
                else: msk_print('Missing filename')
            elif cmd == 'write':
                if len(l) > 0: msk_write(l[0])
                else: msk_print('Missing filename')
            elif cmd == 'writesol':
                if len(l) > 0: msk_writesol(l[0])
                else: msk_print('Missing basename')            
            elif cmd == 'param':
                if len(l) == 2: msk_putparam(l[0], l[1])
                elif len(l) == 1: msk_getparam(l[0])
                else: msk_getparam('')
            elif cmd == 'paramdef':
                msk_paramdef()
            elif cmd == 'paramdiff':
                msk_paramdiff()
            elif cmd == 'info':
                if len(l) == 1: msk_getinfo(l[0])
                else: msk_getinfo('')
            elif cmd == 'anapro':
                msk_anapro_struct_var()
                msk_anapro_struct_con()
                msk_anapro_struct_cones()
                msk_anapro_data()
            elif cmd == 'anasol':
                msk_anasol()
            elif cmd == 'infsub':
                msk_infsub()
            elif cmd == 'delsol':
                msk_delsol()
            elif cmd == 'hist':
                msk_hist()
            elif cmd == 'spy':
                msk_spy()
            elif cmd == 'histsol':
                msk_histsol()
            elif cmd == 'truncate':
                if len(l) > 0: msk_truncate(float(l[0]))
                else: msk_print('Missing epsilon')
            elif cmd == 'resobj':
                if len(l) > 0: msk_resobj(float(l[0]))
                else: msk_resobj(0.0)
            elif cmd == 'optserver':
                if len(l) > 0: msk_optserver(l[0])
                else: msk_optserver("")
            else: msk_print('Unknown command')
        except mosek.Error:
            # Error description is printed to the err stream
            pass
            
# Execute a compound command
def msk_compound_command(line):
    ls = line.split(';')
    for l in ls: msk_command(l.split())

# Run in interactive mode
def msk_interactive_mode():
    while True:
        msk_write_log('MOSEK>>>')
        line = sys.stdin.readline()
        if logfile:
            logfile.write(line)
            logfile.flush()
        msk_compound_command(line)

# Run in batch or interactive mode
if len(sys.argv) == 1:
    msk_interactive_mode()
else:
    msk_compound_command(sys.argv[1] + ';exit')
