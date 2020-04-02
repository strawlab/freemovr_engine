import re

# globals
re_ros_path = re.compile(r'package://(.*?)/')
re_ros_path_old = re.compile(r'\$\(find (.*)\)') # deprecated in favor of above

def _findrepl(matchobj):
    ros_pkg_name = matchobj.group(1)
    return roslib.packages.get_pkg_dir(ros_pkg_name) + os.sep

def fixup_path( orig_path ):
    # Ideally this would use the motmot_ros_utils package,
    # specifically the URL format decoded by rosutils.io.decode_url()
    fixed_path = re_ros_path.sub( _findrepl, orig_path )
    if fixed_path != orig_path:
        return fixed_path
    fixed_path = re_ros_path_old.sub( _findrepl, orig_path )
    if fixed_path != orig_path:
        new_fixed = re_ros_path_old.sub( _suggestrepl, orig_path )
        warnings.warn("a path was specified with %r but "
                      "this is deprecated and should be %r" % (
                      fixed_path, new_fixed))
        return fixed_path
    return orig_path
