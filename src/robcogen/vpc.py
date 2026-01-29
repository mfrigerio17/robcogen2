import re
import sympy

digit_matcher = re.compile("^[0-9]")

def symbolicExpressionToIdentifier(symb_expr):
        s = str(symb_expr)
        s = s.replace(" ", "").replace(".","").replace('*', '_').replace('/', '_')
        if digit_matcher.match(s) :
            s = '_' + s
        return s

def symbolicExpressionToCode(symb_expr, replacements, printer):
    '''
    Arguments:
      - `symb_expr`: a Sympy expression
      - `replacements` : a map from symbol to string, for each Sympy symbol in
        the given expression. Must have the `items()` method, like python
        dictionaries or Lua tables (when using Lupa)
      - `printer`: a function to turn a Sympy expression into text in a target
        language. Typically a member of `sympy.printing`, like `cxxcode`
    '''
    return printer(
          symb_expr.subs(
              {s:sympy.Symbol(name=txt) for s,txt in replacements.items()}
        ))
    # 'items()' will work for both a python dictionary and a Lua table.

    # What I need, is really just printing the expression while replacing every
    # occurrence of a symbol with the given text.
    # I could not find an easier way than doing actual symbols replacement with
    # 'subs()', combined with the hack of naming the replacing symbol as the
    # text that is needed.
    # I cannot use the text itself (which would normally work), because such
    # text may be something like "struct.field", which makes Sympy raise an
    # error.
    # There is NO printing option to replace a symbol with custom text (to
    # avoid subs() in the first place)
