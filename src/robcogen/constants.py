import numbers
import sympy
import robcogen.vpc as expr_utils
import kgprim.values as expr


class ConstantsAccess:
    '''
    A generator of the expressions (in a target language) that would resolve to
    the (cached) value of a model constant.

    This class only deals with the constant-folding option: if enabled, the
    expression for a constant is the number literal corresponding to its value.
    Otherwise, a language-backend-specific configurator is queried for the
    expression to use.

    For example, a language-backend may want to host the model constants inside
    some constant variables in the target language (in a specific container,
    in a namespace, etc.). In that case, the configurator given to this
    class should return the expression in the target language that would resolve
    to a read-access to such variables (e.g. the namespace-qualified name of the
    variable holding the constant value).

    This class is a helper for generating code that has to _read_ the constant
    values (for whatever reason). It is not a helper for generating the code
    that defines the constants themselves. In fact, the configurator given to
    this class must be consistent with the code that defines the constants.
    '''

    def __init__(self, lang, constantFolding=False):
        self.lang  = lang
        self.cfold = constantFolding

    @property
    def isConstantFolding(self):
        return self.cfold

    def valueExpression(self, constant) :
        # if the input itself is a float we cannot do anything but return it
        if isinstance(constant, numbers.Number) :
            return str(constant)

        value = constant.value
        if value == 0.0 or self.isConstantFolding :
            return str(value)

        if isinstance(constant, expr.MyPI)  :
            # We always fold in the numerical value of PI expressions
            return lang.piExpression()

        # The property is necessarily a non-zero constant, and we do not want
        # constant folding; therefore return the expression that resolves to
        # its value
        return self.lang.valueExpression(constant)


# # DEPRECATED @DEPRECATED
# def constantsDefinitions(constantsMetadata, lang):
#     '''
#     A helper function to generate a list of variables defined with the model
#     constants.
# 
# 
#     See the explanation example in the docs of `ConstantsAccess`.
#     '''
#     listing = []
#     replacements = { sympy.sin:lang.sin_func(), sympy.cos:lang.cos_func() }
#     for cc in constantsMetadata :
#         cc_symbol = cc.quantity.sym
#         cc_var, code = lang.value_definition( cc )
#         listing.append( code )
#         replacements[cc_symbol] = cc_var
#         for expr in cc.expressions :
#             sympy_expr = expr.symbolic()
#             if expr.isRotation() :
#                 aux = sympy.sin( sympy_expr )
#                 val = expr_utils.symbolicExpressionToCode(aux, replacements )
#                 code = lang.expr_value_definition( aux, val )
#                 listing.append( code )
#                 aux = sympy.cos( sympy_expr )
#                 val = expr_utils.symbolicExpressionToCode(aux, replacements )
#                 code = lang.expr_value_definition( aux, val )
#                 listing.append( code )
#             else:
#                 # If the expression is the identity, we do not want to generate code
#                 # for it, because we already did it above for the constant
#                 # itself. We check this condition by comparing the Sympy expression
#                 # associated with the Constant and the one associated with the
#                 # Expression. If they are the same, the Expression is a trivial
#                 # identity.
#                 if cc_symbol != sympy_expr :
#                     val = expr_utils.symbolicExpressionToCode(sympy_expr, replacements )
#                     listing.append( lang.expr_value_definition( sympy_expr, val ) )
#         replacements[cc_symbol] = None #clear the entry
#     return listing
