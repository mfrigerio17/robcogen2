import re
import sympy

digit_matcher = re.compile("^[0-9]")

def symbolicExpressionToIdentifier(symb_expr):
        s = str(symb_expr)
        s = s.replace(" ", "").replace(".","").replace('*', '_').replace('/', '_')
        if digit_matcher.match(s) :
            s = '_' + s
        return s

def symbolicExpressionToCode(symb_expr, replacements):
    '''
    - `symb_expr`: a Sympy expression
    '''
    subs = {}
    for k,v in replacements.items() :
        subs[k] = sympy.Symbol(name=v)
    val = symb_expr.subs( subs )
    return str(val)





