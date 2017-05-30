# Steps I needed to follow to release this package

1. Register [on PyPI](https://pypi.python.org/pypi?%3Aaction=register_form).

2. Create a [setup.py](setup.py).

3. Create a wheel for easy installation (given is pure python without dependences and working on Python 2 and 3): `python setup.py bdist_wheel`

4. Install updating all dependences twine: `sudo pip install twine --upgrade` `sudo pip install --upgrade pyOpenSSL` 

5. Create [setup.cfg](setup.cfg) for declaring it a universal package.

6. Create [~/.pypirc](.pypirc) file with my credentials:
```
[pypi]
username = <username>
password = <password>
```

7. Upload the package: `twine upload dist/*`

