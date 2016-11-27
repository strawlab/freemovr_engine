
from .dlt import (direct_linear_transform,
                  hartley_gold_algorithm)
from .pnp import (pnp_iterative,
                  pnp_p3p,
                  pnp_epnp,
                  pnp_ransac_iterative,
                  pnp_ransac_p3p,
                  pnp_ransac_epnp)

ExtrinsicsAlgorithms = {
    'direct_linear_transform': {
        'function': direct_linear_transform,
        'parameters': direct_linear_transform.params,
        'description': ("An implementation of the DLT algorithm based on Hartley Zisserman. "
                        "Does not require intrinsics. Does not use an initial value. "
                        "Provide with at least 25 point 2 point correspondances."),
        'requires-intrinsics': False
    },
    'hartley_gold_algorithm': {
        'function': hartley_gold_algorithm,
        'parameters': hartley_gold_algorithm.params,
        'description': ("An implementation of the gold algorithm based on Hartley Zisserman. "
                        "Does not require intrinsics. Can use a start value. If none is "
                        "provided, uses the dlt result as a guess. "
                        "Parameter constraint normalizes either the whole P matrix (0), "
                        "or just ||(p31, p32, p33)|| (1). "
                        "Provide with at least 25 point 2 point correspondances."),
        'requires-intrinsics': False
    },
    'pnp_iterative': {
        'function': pnp_iterative,
        'parameters': pnp_iterative.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
    'pnp_p3p': {
        'function': pnp_p3p,
        'parameters': pnp_p3p.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
    'pnp_epnp': {
        'function': pnp_epnp,
        'parameters': pnp_epnp.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
    'pnp_ransac_iterative': {
        'function': pnp_ransac_iterative,
        'parameters': pnp_ransac_iterative.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
    'pnp_ransac_p3p': {
        'function': pnp_ransac_p3p,
        'parameters': pnp_ransac_p3p.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
    'pnp_ransac_epnp': {
        'function': pnp_ransac_epnp,
        'parameters': pnp_ransac_epnp.params,
        'description': ("TODO"),
        'requires-intrinsics': True
    },
}






