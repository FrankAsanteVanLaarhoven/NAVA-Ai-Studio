# Describes a constraint parsed from the intent DSL
string kind            # e.g., "approach_from", "keep_out", "cone"
float32[] params       # free-form numeric params (documented per kind)
string frame           # frame of reference, e.g., "map" or "base_link"
