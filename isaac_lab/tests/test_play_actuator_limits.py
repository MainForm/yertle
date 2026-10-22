"""Verify opt-in play limits without launching Kit or changing legacy defaults."""
import argparse
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

TREE=ast.parse((Path(__file__).resolve().parents[1]/'play.py').read_text(encoding='utf8'))

class PlayLimitTests(unittest.TestCase):
    def test_cli_validation(self):
        parser=argparse.ArgumentParser()
        for flag in ('--effort-limit-sim','--velocity-limit-sim'):
            node=next(n for n in TREE.body if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and any(isinstance(a,ast.Constant) and a.value==flag for a in n.value.args))
            exec(compile(ast.Module(body=[node],type_ignores=[]),'play.py','exec'),{'parser':parser})
        args=parser.parse_args(['--effort-limit-sim','.64','--velocity-limit-sim','2.3'])
        self.assertEqual((args.effort_limit_sim,args.velocity_limit_sim),(.64,2.3))
        validation=next(n for n in TREE.body if isinstance(n,ast.For) and isinstance(n.target,ast.Name) and n.target.id=='name' and 'effort_limit_sim' in ast.unparse(n.iter))
        for field in ('effort_limit_sim','velocity_limit_sim'):
            for value in (0.,-1.,float('nan'),float('inf')):
                args=NS(effort_limit_sim=None,velocity_limit_sim=None);setattr(args,field,value)
                with patch.object(parser,'error',side_effect=ValueError),self.assertRaises(ValueError):
                    exec(compile(ast.Module(body=[validation],type_ignores=[]),'play.py','exec'),{'parser':parser,'args_cli':args})

    def test_assignments_precede_environment_creation_and_defaults_are_unchanged(self):
        main=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        overrides=[n for n in main.body if isinstance(n,ast.If) and ast.unparse(n.test) in ('args_cli.effort_limit_sim is not None','args_cli.velocity_limit_sim is not None')]
        creation=next(i for i,n in enumerate(main.body) if isinstance(n,ast.Assign) and any(ast.unparse(t)=='env' for t in n.targets))
        self.assertEqual(len(overrides),2)
        self.assertTrue(all(main.body.index(n)<creation for n in overrides))
        code=compile(ast.Module(body=overrides,type_ignores=[]),'play.py','exec')
        for args,expected in [(NS(effort_limit_sim=None,velocity_limit_sim=None),(1.9,4.6)),(NS(effort_limit_sim=.64,velocity_limit_sim=2.3),(.64,2.3))]:
            legs=NS(effort_limit_sim=1.9,velocity_limit_sim=4.6)
            exec(code,{'legs':legs,'args_cli':args})
            self.assertEqual((legs.effort_limit_sim,legs.velocity_limit_sim),expected)

if __name__=='__main__':unittest.main()
