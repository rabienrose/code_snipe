<?php
$file = 'uploads/chamo_'.date('Y-m-d-H-i-s').'.txt';
$request_body = @file_get_contents('php://input');
file_put_contents($file, $request_body);
var_dump($request_body);
?>