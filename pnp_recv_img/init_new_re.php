<?php
foreach (getallheaders() as $name => $value) {
    echo "$name: $value\n";
}
$file = 'uploads/chamo_'.date('Y-m-d-H-i-s');
mkdir($file);
file_put_contents('global/cur_task.txt', $file);
?>