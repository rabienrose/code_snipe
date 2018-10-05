<?php
$headers = getallheaders();
$my_task=$headers['my_task'];
$my_type=$headers['my_type'];
$my_ind=$headers['my_ind'];
if (!file_exists('uploads')) {
    mkdir('uploads');
}
if (!file_exists('uploads/'.$my_task)) {
    mkdir('uploads/'.$my_task);
}
if ($my_type =='img'){
    if (!file_exists('uploads/'.$my_task.'/images')) {
        mkdir('uploads/'.$my_task.'/images');
    }
    $file = 'uploads/'.$my_task.'/images/'.$my_ind.'.jpg';
}elseif ($my_type =='video'){
    $file = 'uploads/'.$my_task.'/'.$my_type.'.mp4';
}else{
    $file = 'uploads/'.$my_task.'/'.$my_type.'.txt';
}

$request_body = @file_get_contents('php://input');
file_put_contents($file, $request_body);
var_dump($request_body);
?>