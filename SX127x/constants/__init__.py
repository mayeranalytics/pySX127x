def add_lookup(cls):
    """ A decorator that adds a lookup dictionary to the class. """
    varnames = filter(str.isupper, cls.__dict__.keys())
    lookup = dict(map(lambda varname: (cls.__dict__.get(varname, None), varname), varnames))
    setattr(cls, 'lookup', lookup)
    return cls

# import submodules AFTER add_lookup exists in the package namespace
from . import LoRa
from . import FSK

__all__ = ["add_lookup", "LoRa", "FSK"]