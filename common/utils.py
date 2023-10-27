from pydrake.multibody.parsing import Parser

def ConfigureParser(parser: Parser):
    """Add the models/package.xml index to the given Parser."""
    package_xml = "models/package.xml"
    parser.package_map().AddPackageXml(filename=package_xml)