"""
Test template for unit testing with pytest
"""

# import what you need from within the package
import pytest

from auv.cv import template

import logging


def test_cv_function():
    """
    Test a cv function withing the cv template file
    """

    # creates a cv template object to test the function
    cv_temp = template.TemplateCV()
    assert template.some_cv_function(cv_temp) == True
