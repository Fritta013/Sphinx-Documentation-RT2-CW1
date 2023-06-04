# Configuration file for the Sphinx documentation builder.


# -- Project information -----------------------------------------------------

import os
import sys

sys.path.insert(0, os.path.abspath('.'))


project = 'RT2_CW2_sphinx'
copyright = '2023, Ines Haouala'
author = 'Ines Haouala'
release = '0.0.1'

# -- General configuration ---------------------------------------------------

extensions = [ 'sphinx.ext.autodoc',
'myst_parser',]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
autodoc_mock_imports = ['rospy', 'ros']
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}



# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']



# -- Options for Autodoc extension -------------------------------------------

autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

# Add the path to my Python scripts directory
sys.path.insert(0, os.path.abspath('./scripts'))
